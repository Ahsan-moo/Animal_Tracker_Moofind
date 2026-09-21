/* =============================================================================
 * gnss_fn.c
 *
 * =============================================================================
 * FIXES IN THIS REVISION  (rev C)
 * =============================================================================
 *
 * [6] THE RADIO-HOLD BUDGET WAS COUNTED WRONG.
 *     held_ms was incremented by GNSS_FIX_POLL_PERIOD_MS (10 s) per poll,
 *     but each poll ALSO spends up to 15 s inside lwm2m_read_raw() before
 *     the sleep. Worst case is 25 s per iteration, so a 30-poll budget
 *     could hold the cellular link down for up to 750 s while reporting
 *     300 s. GNSS_LAST_HELD_RADIO_MS() -- which main.c uses to decide how
 *     long to wait for re-attach -- was therefore under-reporting by up to
 *     2.5x, which is a plausible contributor to the LWM2M client stalling
 *     at clStatus 3 after a long GNSS cycle.
 *
 *     The loop is now bounded by MEASURED WALL-CLOCK TIME against
 *     GNSS_FIX_BUDGET_MS, not by a poll count, and held_ms is measured
 *     rather than assumed. The poll cap is retained only as a backstop.
 *
 * [7] FIX BUDGET IS NOW ADAPTIVE, AND JUSTIFIED BY MEASURED DATA.
 *     is_xtra_active() is already evaluated before the poll loop but its
 *     result was only logged. It now selects the budget: a warm start does
 *     not need five minutes, and holding the radio down for five minutes
 *     when the answer arrives in twenty seconds is pure loss.
 *
 *     Evidence -- platform log 02-04 Sep 2026, 130 consecutive cycles. The
 *     server-side inter-registration period quantises in exactly 10 s steps
 *     (this loop's period), so the poll count per cycle can be read
 *     directly out of it:
 *         0 extra polls : 32 cycles      4 polls   : 11 cycles
 *         1 poll        : 30 cycles      6-8 polls : 10 cycles
 *         2 polls       : 17 cycles      10-18     :  5 cycles
 *         3 polls       : 22 cycles
 *     Every one of the 130 cycles obtained a timestamp; the worst was 18
 *     polls (~180 s) and NONE approached the 30-poll budget. A median of
 *     1-2 polls is warm-start behaviour, so XTRA was in fact active during
 *     that run, contrary to the "XTRA has since lapsed" note below -- most
 *     likely FIX [5] repaired it. GNSS_FIX_BUDGET_XTRA_MS (120 s) covers
 *     125 of those 130 cycles; the cold-start budget covers the rest.
 *
 * [8] A NON-ZERO TIMESTAMP IS NOT NECESSARILY A FRESH FIX.
 *     The success test was `obj_out[0] > 0`, which any stale value left in
 *     the agent's location object also satisfies. A stale value would
 *     return on poll 0 every single cycle; the observed 0-18 spread argues
 *     it was genuinely fresh, but the test is still wrong. The timestamp is
 *     now required to be >= RTC_SANE_EPOCH, and, when the RTC is itself
 *     already network-set, to be within GNSS_FIX_MAX_AGE_S of it.
 *     NOTE: the age-against-RTC half of this was WRONG and is superseded
 *     by FIX [11] below -- it rejected every live fix in any non-UTC
 *     timezone. The >= RTC_SANE_EPOCH floor is retained.
 *
 * [11] THE FRESHNESS TEST REJECTED EVERY REAL FIX.
 *     See the long note at GNSS_FIX_MAX_FUTURE_S. Freshness is now judged
 *     by whether the timestamp ADVANCES between polls, which does not
 *     depend on the RTC and the GNSS receiver agreeing on a timeline.
 *
 * [9] RESTORE TIME IS NOW COUNTED.
 *     restore_wwan_priority() can spend up to 30 s retrying, and none of it
 *     was included in held_ms even though the link is down for all of it.
 *
 * [10] RTC_SANE_EPOCH IS DUPLICATED IN sleep_fn.c.
 *     Both files need it and neither owns it.
 *     TODO: move to app_cfg.h and include from both.
 *
 * -----------------------------------------------------------------------------
 * FIXES CARRIED FORWARD FROM THE PREVIOUS REVISION
 * -----------------------------------------------------------------------------
 * [1] GNSS held the radio for up to 5 minutes and told nobody.
 *     m2mb_gnss_set_prio_runtime(GNSS_PRIORITY) takes the cellular link
 *     down on this shared-RF part for as long as the fix attempt runs.
 *     GNSS_ROUTINE records the hold and the timeout outcome, exposed via
 *     GNSS_LAST_HELD_RADIO_MS() and GNSS_LAST_TIMED_OUT().
 * [2] Fix budget: cut to 12 (2 min), then restored to 30 (5 min).
 *     Superseded by FIX [7] -- it is now selected from XTRA state.
 * [3] Priority is always restored; all exits go through one path.
 * [4] XTRA deadlock broken -- see the note at step 6.
 * Preserved: bounded init/priority retries, static aligned obj buffer (no
 * malloc leak), callback reads fields only, RTC-from-GNSS guard.
 *
 * -----------------------------------------------------------------------------
 * BUILD NOTE: an older stub header named GNSS_FN.h (uppercase) exists in
 * some trees, dated 12.05.2026, with `//void GNSS_ROUTINE();` commented
 * out. On a case-insensitive filesystem it collides with gnss_fn.h and the
 * toolchain may resolve includes to the stub, producing
 * "'GNSS_ROUTINE' was not declared in this scope" in main.c. DELETE the
 * uppercase GNSS_FN.h; this file pairs with lowercase gnss_fn.h only.
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "m2mb_types.h"
#include "m2mb_os_api.h"
#include "m2mb_gnss.h"
#include "m2mb_lwm2m.h"
#include "m2mb_rtc.h"

#include "azx_log.h"
#include "azx_utils.h"

#include "app_common.h"
#include "at_helper.h"
#include "lwm2m_fn.h"
#include "gnss_fn.h"

/* FIX [7]: the budget is chosen from XTRA state rather than fixed.
 *
 * XTRA active  -> warm start. 120 s covered 125 of the 130 measured cycles.
 * XTRA lapsed  -> cold start: the receiver must decode ephemeris off the
 *                 satellites, which takes 30 s to several minutes with
 *                 clear sky and effectively never completes indoors. */
#define GNSS_FIX_BUDGET_XTRA_MS   (120 * 1000)   /* warm start */
#define GNSS_FIX_BUDGET_COLD_MS   (300 * 1000)   /* cold start */

#define GNSS_FIX_POLL_PERIOD_MS   10000
#define GNSS_FIX_READ_TIMEOUT_MS  15000

/* Backstop only. The real bound is the wall-clock budget above; this just
 * stops a runaway loop if the time source misbehaves. */
#define GNSS_FIX_POLL_MAX         40

#define RTC_SANE_EPOCH            1704067200 /* 2024-01-01: RTC older than
                                              * this was never network-set */

/* FIX [11]: FRESHNESS IS NOW JUDGED BY MOVEMENT, NOT BY AGE-AGAINST-RTC.
 *
 * The previous test compared the fix timestamp against the module RTC and
 * rejected anything further away than GNSS_FIX_MAX_AGE_S (3600 s). That
 * assumed both clocks share a timeline. They do not.
 *
 * Observed on 07 Sep 2026, Hannover (CEST = UTC+2):
 *     fix timestamp 1788768819 is 7200 s from RTC -- stale, ignoring
 *     fix timestamp 1788768831 is 7198 s from RTC -- stale, ignoring
 *     fix timestamp 1788768840 is 7199 s from RTC -- stale, ignoring
 *     fix timestamp 1788768849 is 7200 s from RTC -- stale, ignoring
 *
 * Those four timestamps decode to 08:13:39, 08:13:51, 08:14:00 and
 * 08:14:09 UTC -- advancing 12 s, 9 s and 9 s apart across consecutive
 * polls. They were live fixes. Every one was discarded.
 *
 * The module RTC holds LOCAL time as an epoch value while GNSS reports
 * true UTC, so the difference is a permanent whole-hour timezone offset:
 * 7200 s in CEST, 3600 s in CET. With the tolerance at 3600 s, EVERY fix
 * is rejected in summer and the winter case sits exactly on the boundary.
 * The device could never obtain a fix in any non-UTC timezone.
 *
 * The fix does not widen the tolerance -- that would only mask genuinely
 * stale values, which is what FIX [8] existed to catch. Instead we use the
 * property that actually distinguishes the two cases:
 *
 *     a leftover value in the agent's location object is STATIC;
 *     a live fix ADVANCES on every poll.
 *
 * This is immune to any clock offset because it never compares the two
 * clocks at all. It costs one extra poll on the first fix of a cycle,
 * because there is no previous sample to compare against.
 *
 * The absolute sanity floor (>= RTC_SANE_EPOCH) is retained: a timestamp
 * predating 2024 is not a real fix regardless of what it does next.
 *
 * GNSS_FIX_MAX_FUTURE_S guards the opposite direction. A GNSS timestamp
 * should never be far ahead of the RTC's own timeline plus the largest
 * real timezone offset (UTC+14 = 50400 s); anything beyond that is a
 * corrupt read rather than a fix.
 *
 * ROOT CAUSE STILL OPEN: something is writing local time into the RTC as
 * if it were UTC. maybe_set_rtc_from_gnss() below writes correct UTC, so
 * the offset is being introduced by NITZ applying the timezone. That also
 * means sleep_fn.c's absolute wake alarms and any timestamp this device
 * reports are two hours out. Worth chasing separately.
 */
#define GNSS_FIX_MAX_FUTURE_S     50400   /* UTC+14, the largest real offset */

static M2MB_GNSS_HANDLE gnss_handle = NULL;

/* FIX [1]: how long the last GNSS_ROUTINE held the radio, and why it ended */
static UINT32  g_gnss_held_ms    = 0;
static BOOLEAN g_gnss_timed_out  = FALSE;

UINT32 GNSS_LAST_HELD_RADIO_MS(void)
{
    return g_gnss_held_ms;
}

BOOLEAN GNSS_LAST_TIMED_OUT(void)
{
    return g_gnss_timed_out;
}

/* ---- measured elapsed time (FIX [6]) -------------------------------------- */
/*
 * Millisecond counter from the RTOS tick, so the radio-hold budget reflects
 * time actually spent rather than time we assumed each iteration would take.
 *
 * NOTE: wraps with the tick counter. All uses below are short differences
 * taken within a single GNSS_ROUTINE call, which unsigned subtraction
 * handles correctly across a wrap.
 */
static UINT32 now_ms(void)
{
    return (UINT32)(m2mb_os_getSysTicks() * m2mb_os_getSysTickDuration_ms());
}

/* ---- GNSS callback (position/NMEA indications, if the FW emits them) ------ */
static void gnss_callback(M2MB_GNSS_HANDLE h, M2MB_GNSS_IND_E gnss_event,
                          UINT16 resp_size, void *resp, void *userdata)
{
    (void)h; (void)resp_size; (void)userdata;

    switch (gnss_event)
    {
    case M2MB_GNSS_INDICATION_POSITION_REPORT:
    {
        /* read fields inside the callback only; do not store the pointer */
        M2MB_GNSS_POSITION_REPORT_INFO_T *pos =
            (M2MB_GNSS_POSITION_REPORT_INFO_T *)resp;
        AZX_LOG_INFO("gnss position latitude:%f longitude:%f\r\n",
                     pos->latitude, pos->longitude);
        set_ev(EV_GNSS_POS_BIT);
        break;
    }
    case M2MB_GNSS_INDICATION_NMEA_REPORT:
    {
        AZX_LOG_INFO("NMEA: %s\n", (CHAR *)resp);
        break;
    }
    default:
        break;
    }
}

/* ---- XTRA (AGNSS) helpers via AT ------------------------------------------ */

/* AT$AGNSS? -> $AGNSS: <provider>,<active>,<requested>
 * Returns TRUE if <active> == 1. */
static BOOLEAN is_xtra_active(void)
{
    char resp[256];
    if (!ati_send_and_wait("AT$AGNSS?\r", resp, sizeof(resp), 5000))
        return FALSE;

    char *p = strstr(resp, "$AGNSS:");
    if (p == NULL)
        p = strstr(resp, "$GPSAGNSS:");   /* some FW versions */
    if (p == NULL)
        return FALSE;

    p = strchr(p, ':');
    if (p == NULL)
        return FALSE;
    p++;

    int prov = -1, active = -1, requested = -1;
    if (sscanf(p, " %d,%d,%d", &prov, &active, &requested) >= 2)
    {
        AZX_LOG_INFO("XTRA status: provider=%d active=%d requested=%d\r\n",
                     prov, active, requested);
        return (active == 1) ? TRUE : FALSE;
    }
    return FALSE;
}

/* Persists in NVM; takes effect on next reboot (alarm cycle provides it).
 * FIX [5]: requested whenever XTRA is not active, fix or no fix. */
static void request_xtra_enable(void)
{
    char resp[256];
    AZX_LOG_INFO("Requesting XTRA enable (AT$AGNSS=0,1) ...\r\n");
    if (ati_send_and_wait("AT$AGNSS=0,1\r", resp, sizeof(resp), 5000))
    {
        AZX_LOG_INFO("XTRA enable requested OK (active on next reboot)\r\n");
    }
    else
    {
        AZX_LOG_ERROR("XTRA enable request failed: %s\r\n", resp);
    }
}

/* ---- RTC from GNSS -------------------------------------------------------- */
/*
 * Only writes the RTC when it was never set. Once NITZ has set it, the
 * network is the better source and we leave it alone.
 *
 * This matters more than it looks: sleep_fn.c derives an ABSOLUTE wake
 * alarm from this clock. If it is never set, the alarm is computed from the
 * power-on default epoch and the module may never wake. sleep_fn.c rev C
 * now self-heals that case independently, but getting a real time in here
 * whenever GNSS can supply one is still the better outcome.
 */
static void maybe_set_rtc_from_gnss(UINT32 gnss_epoch)
{
    INT32 rtcfd = m2mb_rtc_open("/dev/rtc0", 0);
    if (rtcfd == -1)
        return;

    M2MB_RTC_TIMEVAL_T tv;
    if (0 == m2mb_rtc_ioctl(rtcfd, M2MB_RTC_IOCTL_GET_TIMEVAL, &tv))
    {
        if (tv.sec < RTC_SANE_EPOCH)
        {
            tv.sec  = (TIME_T)gnss_epoch;
            tv.msec = 0;
            if (0 == m2mb_rtc_ioctl(rtcfd, M2MB_RTC_IOCTL_SET_TIMEVAL, &tv))
            {
                time_t t = (time_t)gnss_epoch;
                struct tm *ptm = gmtime(&t);
                if (ptm != NULL)
                {
                    AZX_LOG_INFO("RTC set from GNSS (UTC): %d-%02d-%02d %02d:%02d:%02d\r\n",
                                 ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
                                 ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
                }
            }
        }
        else
        {
            AZX_LOG_INFO("RTC already set by network, skipping GNSS time\r\n");
        }
    }
    m2mb_rtc_close(rtcfd);
}

/* ---- fix plausibility (FIX [8], reworked by FIX [11]) --------------------- */
/*
 * Previous sample within the current GNSS_ROUTINE call. Reset to 0 at the
 * top of every cycle so a value cannot carry over from the last one.
 */
static UINT32 s_prev_fix_ts = 0;

/*
 * TRUE only if the reported timestamp looks like a real, live fix rather
 * than a leftover value sitting in the agent's location object.
 *
 * Test: has the timestamp CHANGED since the previous poll of this cycle?
 * A stale object value does not move. A live fix advances every poll.
 *
 * See the FIX [11] note at GNSS_FIX_MAX_FUTURE_S for why this replaced the
 * previous age-against-RTC comparison.
 *
 * Returns FALSE on the first poll of a cycle by construction: there is no
 * previous sample yet. That costs one poll period and is the price of not
 * depending on clock agreement.
 */
static BOOLEAN fix_timestamp_is_fresh(UINT32 ts)
{
    UINT32 prev = s_prev_fix_ts;

    /* Absolute floor: a timestamp predating 2024 is not a fix. */
    if (ts < (UINT32)RTC_SANE_EPOCH)
    {
        AZX_LOG_INFO("fix timestamp %lu predates the sane epoch -- ignoring\r\n",
                     (unsigned long)ts);
        s_prev_fix_ts = 0;
        return FALSE;
    }

    /* Guard the other direction: reject a value implausibly far ahead of
     * the RTC's own timeline. Uses the largest real timezone offset as the
     * allowance so a local-time RTC does not trip it. */
    {
        INT32 fd = m2mb_rtc_open("/dev/rtc0", 0);
        if (fd != -1)
        {
            M2MB_RTC_TIMEVAL_T tv;
            if (0 == m2mb_rtc_ioctl(fd, M2MB_RTC_IOCTL_GET_TIMEVAL, &tv) &&
                tv.sec >= (TIME_T)RTC_SANE_EPOCH)
            {
                INT32 ahead = (INT32)((TIME_T)ts - tv.sec);
                if (ahead > GNSS_FIX_MAX_FUTURE_S)
                {
                    AZX_LOG_INFO("fix timestamp %lu is %ld s AHEAD of the RTC "
                                 "-- implausible, ignoring\r\n",
                                 (unsigned long)ts, (long)ahead);
                    m2mb_rtc_close(fd);
                    s_prev_fix_ts = ts;
                    return FALSE;
                }
            }
            m2mb_rtc_close(fd);
        }
    }

    /* The actual freshness test: did it move? */
    s_prev_fix_ts = ts;

    if (prev == 0)
    {
        AZX_LOG_INFO("fix timestamp %lu seen; waiting one more poll to "
                     "confirm it is advancing\r\n", (unsigned long)ts);
        return FALSE;
    }

    if (ts == prev)
    {
        AZX_LOG_INFO("fix timestamp %lu unchanged since last poll -- not a "
                     "live fix yet\r\n", (unsigned long)ts);
        return FALSE;
    }

    AZX_LOG_INFO("fix timestamp advancing (%lu -> %lu, +%ld s) -- live fix\r\n",
                 (unsigned long)prev, (unsigned long)ts,
                 (long)((INT32)(ts - prev)));
    return TRUE;
}

/* ---- priority restore (FIX [3]: single exit path) ------------------------- */
static void restore_wwan_priority(UINT32 *held_ms)
{
    int ctr = 0;
    UINT32 t0 = now_ms();

    if (gnss_handle == NULL)
        return;

    while (M2MB_RESULT_SUCCESS != m2mb_gnss_set_prio_runtime(gnss_handle,
                                                             WWAN_PRIORITY))
    {
        AZX_LOG_INFO("m2mb_gnss_set_prio_runtime_wwan failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 30)
        {
            AZX_LOG_ERROR("Could not restore WWAN priority -- the cellular "
                          "link will stay down this cycle\r\n");
            break;
        }
    }

    if (ctr <= 30)
        AZX_LOG_INFO("m2mb_gnss_set_prio_runtime_wwan succeeded\r\n");

    /* FIX [9]: the link is down for all of the above; count it. */
    if (held_ms != NULL)
        *held_ms += (now_ms() - t0);
}

/* ---- main GNSS routine ---------------------------------------------------- */
BOOLEAN GNSS_ROUTINE(void)
{
    int ctr;
    BOOLEAN gnss_fix_obtained = FALSE;
    UINT32  held_ms  = 0;
    UINT32  budget_ms;
    UINT32  t_hold_start;

    /* static aligned buffer instead of unchecked 10-byte malloc leak */
    static UINT32 obj_out[4];

    g_gnss_held_ms   = 0;
    g_gnss_timed_out = FALSE;

    /* FIX [11]: no previous sample at the start of a cycle. */
    s_prev_fix_ts    = 0;

    /* -- 1. Init GNSS -- */
    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_gnss_init(&gnss_handle,
                                                 gnss_callback, NULL))
    {
        AZX_LOG_INFO("m2mb_gnss_init failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 10)
            return FALSE;          /* radio never taken, nothing to restore */
    }
    AZX_LOG_INFO("m2mb_gnss_init succeeded\r\n");

    /* -- 2. Check XTRA status; FIX [7]: it now selects the fix budget -- */
    BOOLEAN xtra_already_active = is_xtra_active();
    if (xtra_already_active)
    {
        budget_ms = GNSS_FIX_BUDGET_XTRA_MS;
        AZX_LOG_INFO("XTRA is active -- warm start expected, budget %lu s\r\n",
                     (unsigned long)(budget_ms / 1000UL));
    }
    else
    {
        budget_ms = GNSS_FIX_BUDGET_COLD_MS;
        AZX_LOG_INFO("XTRA not yet active -- cold start this cycle, "
                     "budget %lu s, will enable after this attempt\r\n",
                     (unsigned long)(budget_ms / 1000UL));
    }

    /* -- 3. Switch modem to GNSS priority --
     * From here on the cellular link is DOWN until restore_wwan_priority(). */
    t_hold_start = now_ms();
    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_gnss_set_prio_runtime(gnss_handle,
                                                             GNSS_PRIORITY))
    {
        AZX_LOG_INFO("m2mb_gnss_set_prio_runtime_gnss failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 30)
        {
            AZX_LOG_ERROR("Could not switch to GNSS priority\r\n");
            /* FIX [3]: the switch may have partially taken -- restore */
            held_ms = now_ms() - t_hold_start;
            restore_wwan_priority(&held_ms);
            g_gnss_held_ms   = held_ms;
            g_gnss_timed_out = TRUE;
            return FALSE;
        }
    }
    AZX_LOG_INFO("m2mb_gnss_set_prio_runtime_gnss succeeded "
                 "(cellular link down from here)\r\n");

    /* -- 4. Object 33211 enables location data storage at object 6 -- */
    {
        M2MB_LWM2M_OBJ_URI_T uri_storage = { 4, 33211, 0, 0, 0 };
        if (!lwm2m_write_u32(&uri_storage, 1))
            AZX_LOG_ERROR("Failed to enable location storage (33211)\r\n");
    }

    /* -- 5. Poll /6/0/5 (timestamp) for a fix --
     *
     * FIX [6]: bounded by measured wall-clock time, not by poll count. Each
     * iteration can cost up to GNSS_FIX_READ_TIMEOUT_MS + POLL_PERIOD_MS,
     * so a count-based bound under-states the radio-down time by up to 2.5x.
     */
    ctr = 0;
    while ((now_ms() - t_hold_start) < budget_ms && ctr < GNSS_FIX_POLL_MAX)
    {
        M2MB_LWM2M_OBJ_URI_T uri_time = { 4, 6, 0, 5, 0 };

        obj_out[0] = 0;
        if (lwm2m_read_raw(&uri_time, obj_out, sizeof(obj_out),
                           GNSS_FIX_READ_TIMEOUT_MS))
        {
            /* FIX [8]: non-zero is not the same as fresh */
            if (obj_out[0] > 0 && fix_timestamp_is_fresh(obj_out[0]))
            {
                AZX_LOG_INFO("fix timestamp: %lu (after %lu s, %d polls)\r\n",
                             (unsigned long)obj_out[0],
                             (unsigned long)((now_ms() - t_hold_start) / 1000UL),
                             ctr);
                gnss_fix_obtained = TRUE;
                maybe_set_rtc_from_gnss(obj_out[0]);
                break;
            }
            /* quieter than logging "fix timestamp: 0" on every poll */
            if ((ctr % 3) == 0)
                AZX_LOG_INFO("waiting for fix... (%d polls, %lu/%lu s)\r\n",
                             ctr + 1,
                             (unsigned long)((now_ms() - t_hold_start) / 1000UL),
                             (unsigned long)(budget_ms / 1000UL));
        }
        else
        {
            AZX_LOG_INFO("lwm2m read of /6/0/5 timed out/failed\r\n");
        }

        /* Do not sleep past the budget. */
        if ((now_ms() - t_hold_start) + GNSS_FIX_POLL_PERIOD_MS >= budget_ms)
            break;

        azx_sleep_ms(GNSS_FIX_POLL_PERIOD_MS);
        ctr++;
    }

    if (!gnss_fix_obtained)
    {
        g_gnss_timed_out = TRUE;
        AZX_LOG_INFO("GNSS: no fix within %lu s (%d polls)\r\n",
                     (unsigned long)((now_ms() - t_hold_start) / 1000UL), ctr);
    }

    /* -- 6. Enable XTRA if not already active --
     *
     * FIX [5]: THIS WAS A DEADLOCK. The old condition was
     *     if (gnss_fix_obtained && !xtra_already_active)
     * i.e. XTRA was only ever requested AFTER a successful fix. But
     * without XTRA the device cold-starts, and a cold start frequently
     * fails to fix inside the budget -- so XTRA never gets requested and
     * the device can never climb out on its own. Observed directly:
     * AT$AGNSS? went from active=1,requested=1 back to active=0,
     * requested=0 and every subsequent cycle failed to fix.
     *
     * The fix-gate served no purpose: AT$AGNSS=0,1 only writes a setting
     * to NVM that takes effect on the next boot. It costs one AT command
     * and does not require a fix to be valid. Request it whenever XTRA is
     * not already active, fix or no fix. */
    if (!xtra_already_active)
    {
        AZX_LOG_INFO("XTRA not active -- requesting enable (fix=%s)\r\n",
                     gnss_fix_obtained ? "yes" : "no");
        request_xtra_enable();
    }

    /* -- 7. Disable location storage, restore WWAN priority -- */
    {
        M2MB_LWM2M_OBJ_URI_T uri_storage = { 4, 33211, 0, 0, 0 };
        lwm2m_write_u32(&uri_storage, 0);
    }

    held_ms = now_ms() - t_hold_start;      /* FIX [6]: measured, not assumed */
    restore_wwan_priority(&held_ms);        /* FIX [9]: counts its own time  */

    g_gnss_held_ms = held_ms;
    AZX_LOG_INFO("GNSS held the radio for ~%lu s (fix=%s, timed_out=%s)\r\n",
                 (unsigned long)(held_ms / 1000UL),
                 gnss_fix_obtained ? "yes" : "no",
                 g_gnss_timed_out ? "yes" : "no");

    return gnss_fix_obtained;
}
