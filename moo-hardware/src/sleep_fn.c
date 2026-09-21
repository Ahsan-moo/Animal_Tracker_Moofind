/* =============================================================================
 * SLEEP_FN.c
 * Alarm-based shutdown cycle, plain shutdown, and reboot.
 *
 * =============================================================================
 * FIXES IN THIS REVISION  (rev C)
 * =============================================================================
 *
 * [6] ROOT CAUSE OF "MODULE NEVER WAKES UP AGAIN".
 *     SET_ALARM_IN() programs an ABSOLUTE date/time alarm derived from the
 *     module RTC, and nothing checked that the RTC was valid.
 *
 *     On a cycle where the network never attaches, NITZ never arrives, so
 *     the clock is never set. GNSS also cannot set it, because
 *     maybe_set_rtc_from_gnss() only runs on a successful fix. The alarm is
 *     therefore computed from the power-on default epoch and programmed at
 *     e.g. 1970-01-01 00:20:00 -- a date the PMU either rejects or treats
 *     as already past. SHUTDOWN() then powers the module off with NO
 *     PENDING ALARM and it stays off until ON/OFF# is pulsed by hand.
 *
 *     Note the trap: the failure path that exists to recover from a failed
 *     network cycle is the one that most depends on a clock a failed
 *     network cycle never sets.
 *
 *     FIX: if the RTC is below RTC_SANE_EPOCH, force it to RTC_SANE_EPOCH
 *     before arming. We do not need the correct wall-clock time to sleep
 *     900 s -- we only need "now" and "alarm" to sit on the same timeline.
 *     Real time is corrected later by NITZ or by GNSS. The device keeps
 *     cycling either way, which is the whole point.
 *
 * [7] RETURN-CODE HANDLING WAS UNSAFE.
 *     Three ioctl results were OR-ed into one `ret` and the caller tested
 *     `!= -1`. That is only correct if every API returns exactly -1 on
 *     failure. Several m2mb APIs return positive M2MB_RESULT_E values
 *     instead, in which case `ret` becomes positive, the `!= -1` test
 *     PASSES, the retry loop never runs, and the module shuts down with no
 *     alarm set. Every call is now tested individually and the function
 *     returns strictly 0 (ok) or -1 (fail).
 *
 * [8] ALARM IS NOW READ BACK AND SANITY-CHECKED.
 *     The computed alarm is verified to be in the future relative to the
 *     clock it was derived from. Optional hardware read-back is available
 *     behind SLEEP_VERIFY_ALARM (see below) once you have confirmed the
 *     GET_ALARM_TIME ioctl name in your m2mb_rtc.h.
 *
 * [9] SHUTDOWN() COULD RETURN SILENTLY.
 *     If m2mb_power_init() failed it logged and returned. ALARM_SHUTDOWN_IN
 *     then returned on the assumption that SHUTDOWN() never comes back,
 *     M2MB_main fell off the end, and the device sat awake at full power --
 *     exactly the failure FIX [3] was written to remove, reached through a
 *     different door. SHUTDOWN() now escalates to REBOOT().
 *
 * [10] REBOOT-LOOP GUARD.
 *     The reboot-as-last-resort path in FIX [3] can loop if the RTC is
 *     genuinely dead. With FIX [6] in place this should now be unreachable,
 *     but a bounded in-session counter is added so a single boot cannot
 *     spin. See the TODO about persisting it.
 *
 * -----------------------------------------------------------------------------
 * FIXES CARRIED FORWARD FROM THE PREVIOUS REVISION
 * -----------------------------------------------------------------------------
 * [1] SLEEP_INTERVAL was 360 s, shorter than the network's T3402 attach
 *     backoff of 720 s, so every wake landed inside a backoff window and
 *     the device could never converge. Now 900 s.
 * [2] The macro was `0.1*60*60`, a floating-point expression substituted
 *     into integer time arithmetic. Now a bracketed integer constant.
 * [3] ALARM_SHUTDOWN() used to log and RETURN when SET_ALARM() failed,
 *     leaving the radio up with no alarm pending until the battery died.
 *     Now retried, then reboot as recovery of last resort.
 * [4] SET_ALARM_IN() / ALARM_SHUTDOWN_IN() allow a longer sleep after a
 *     failed network cycle without changing the normal cadence.
 * [5] Removed the unused TIME_WAKEUP define.
 *
 * -----------------------------------------------------------------------------
 * MEASURED BEHAVIOUR THIS REVISION IS TUNED AGAINST
 * -----------------------------------------------------------------------------
 * Platform log 02-04 Sep 2026, 130 consecutive cycles:
 *   observed server-side period 1022.7 s, sd 18.6 s
 *   = SLEEP_INTERVAL (900 s) + active window (122.7 s mean, 102-183 s range)
 * The alarm path itself is sound -- it worked 130 times in a row. It fails
 * only under the FIX [6] condition, which never occurred during that soak
 * because the module attached on every single cycle.
 *
 * See also FIX [11]: the alarm is armed at cycle END, so the true period is
 * SLEEP_INTERVAL + active time and drifts with coverage. Anchoring it to
 * the wake instant is a main.c change and is described in the header note
 * below rather than done here.
 * ========================================================================== */
#include "sleep_fn.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "m2mb_types.h"
#include "m2mb_os_api.h"
#include "m2mb_uart.h"
#include "app_cfg.h"
#include "azx_log.h"
#include "azx_utils.h"
#include <time.h>
#include "m2mb_rtc.h"
#include "m2mb_power.h"
#include "m2mb_net.h"
#include "m2mb_pdp.h"
#include "m2mb_socket.h"
#include "m2mb_lwm2m.h"
#include "m2mb_gnss.h"
#include "m2mb_ati.h"

/* -----------------------------------------------------------------------------
 * FIX [1][2]: normal wake cadence.
 *
 * MUST stay above the network's T3402 attach backoff (720 s on this network,
 * readable from field 17 of AT#RFSTS). If you shorten this below T3402 you
 * reintroduce the perpetual "status:2" loop, because every wake will land
 * inside a backoff period the previous cycle triggered.
 * -------------------------------------------------------------------------- */
#define SLEEP_INTERVAL      (15 * 60)    /* 900 s */

/* Longer cadence for use after a failed network cycle. Gives the modem a
 * full backoff period plus margin before the next cold attach attempt. */
#define SLEEP_INTERVAL_FAIL (20 * 60)    /* 1200 s */

/* How many times to retry programming the RTC alarm before giving up. */
#define ALARM_SET_RETRIES   3

/* FIX [6]: a clock below this was never set by NITZ or GNSS and cannot be
 * used to compute an absolute alarm.
 *
 * TODO: this constant is duplicated in gnss_fn.c. Move it to app_cfg.h and
 * include it from both, so the two files can never disagree. */
#define RTC_SANE_EPOCH      1704067200   /* 2024-01-01 00:00:00 UTC */

/* FIX [8]: optional hardware read-back of the programmed alarm.
 *
 * Leave at 0 until you have confirmed the exact GET_ALARM_TIME ioctl name
 * in YOUR m2mb_rtc.h -- it is not identical across SDK versions and a wrong
 * name will not compile. When you enable it, fill in the ioctl below. */
#define SLEEP_VERIFY_ALARM  0

/* FIX [10]: bound the reboot-as-recovery path within one boot session.
 *
 * TODO: this is a RAM counter and is lost across the very reboot it counts,
 * so it cannot stop a true reboot loop on its own. Persist it (small file
 * via m2mb_fs, or an NVM setting) and add exponential back-off before this
 * is relied on in the field. With FIX [6] in place the path should now be
 * unreachable, which is why this is left as a guard rather than a feature. */
#define REBOOT_RECOVERY_MAX 2
static int g_reboot_recovery_count = 0;

void *sleep_userdata = NULL;
INT32 rtcfd;

/* ---- RTC helpers ---------------------------------------------------------- */

/*
 * FIX [6]: make sure the RTC sits on a usable timeline before we derive an
 * absolute alarm from it.
 *
 * Returns 0 if the clock is usable on return (either it was already valid,
 * or we successfully forced it to RTC_SANE_EPOCH), -1 if it could not be
 * made usable at all.
 *
 * Forcing a known-wrong-but-consistent time is deliberate. An absolute
 * alarm only has to be later than "now" on the same timeline; it does not
 * have to be the correct wall-clock time. A device that keeps waking with a
 * wrong clock is recoverable. A device that never wakes is not.
 */
static int rtc_ensure_usable(INT32 fd, M2MB_RTC_TIMEVAL_T *tv)
{
    if (m2mb_rtc_ioctl(fd, M2MB_RTC_IOCTL_GET_TIMEVAL, tv) != 0)
    {
        AZX_LOG_ERROR("RTC: GET_TIMEVAL failed\r\n");
        return -1;
    }

    if (tv->sec >= (TIME_T)RTC_SANE_EPOCH)
        return 0;                       /* already set by NITZ or GNSS */

    AZX_LOG_ERROR("RTC NOT SET (epoch=%lu) -- forcing to sane baseline so the "
                  "wake alarm can still be armed\r\n",
                  (unsigned long)tv->sec);

    tv->sec  = (TIME_T)RTC_SANE_EPOCH;
    tv->msec = 0;

    if (m2mb_rtc_ioctl(fd, M2MB_RTC_IOCTL_SET_TIMEVAL, tv) != 0)
    {
        AZX_LOG_ERROR("RTC: SET_TIMEVAL failed -- cannot arm an absolute "
                      "alarm this cycle\r\n");
        return -1;
    }

    /* read it back; do not trust a write we did not verify */
    if (m2mb_rtc_ioctl(fd, M2MB_RTC_IOCTL_GET_TIMEVAL, tv) != 0)
    {
        AZX_LOG_ERROR("RTC: GET_TIMEVAL after set failed\r\n");
        return -1;
    }

    if (tv->sec < (TIME_T)RTC_SANE_EPOCH)
    {
        AZX_LOG_ERROR("RTC: baseline did not stick (epoch=%lu)\r\n",
                      (unsigned long)tv->sec);
        return -1;
    }

    AZX_LOG_INFO("RTC baseline applied; real time will be corrected by NITZ "
                 "or GNSS on a later cycle\r\n");
    return 0;
}

/* ---- alarm programming ---------------------------------------------------- */

/*
 * Returns 0 on success, -1 on failure.
 *
 * NOTE FOR CALLERS IN OTHER FILES: the failure value is unchanged (-1), so
 * an existing `if (SET_ALARM() != -1)` test still behaves correctly. New
 * code should test `== 0`.
 */
int SET_ALARM_IN(UINT32 seconds)
{
    M2MB_RTC_TIMEVAL_T time_val;
    M2MB_RTC_TIME_T    rtc_time;
    struct tm *ptm;
    time_t     time_val_t;
    TIME_T     now_sec;

    if (seconds == 0)
    {
        AZX_LOG_ERROR("SET_ALARM_IN: refusing a zero-second alarm\r\n");
        return -1;
    }

    rtcfd = m2mb_rtc_open("/dev/rtc0", 0);
    if (rtcfd == -1)
    {
        AZX_LOG_ERROR("Cannot open RTC!\r\n");
        return -1;
    }
    AZX_LOG_INFO("RTC opened\r\n");

    /* FIX [6]: validate, and self-heal if the clock was never set */
    if (rtc_ensure_usable(rtcfd, &time_val) != 0)
    {
        m2mb_rtc_close(rtcfd);
        return -1;
    }
    now_sec = time_val.sec;

    /* FIX [7]: test every call on its own, no OR-accumulated status.
     *
     * rtc_time is populated here so that any fields we do NOT overwrite
     * below (timezone, daylight flag, whatever this SDK carries) keep the
     * values the module itself reported. Only the date/time fields are
     * replaced. */
    if (m2mb_rtc_ioctl(rtcfd, M2MB_RTC_IOCTL_GET_SYSTEM_TIME, &rtc_time) != 0)
    {
        AZX_LOG_ERROR("RTC: GET_SYSTEM_TIME failed\r\n");
        m2mb_rtc_close(rtcfd);
        return -1;
    }

    AZX_LOG_INFO("Module system time is: %d-%02d-%02d, %02d:%02d:%02d "
                 "(epoch %lu)\r\n",
                 rtc_time.year, rtc_time.mon, rtc_time.day,
                 rtc_time.hour, rtc_time.min, rtc_time.sec,
                 (unsigned long)now_sec);

    time_val.sec += (TIME_T)seconds;

    /* FIX [8]: the alarm must be strictly in the future on this timeline.
     * Catches wrap, a bad `seconds`, and any clock that moved under us. */
    if (time_val.sec <= now_sec)
    {
        AZX_LOG_ERROR("Computed alarm (%lu) is not after now (%lu) -- "
                      "refusing\r\n",
                      (unsigned long)time_val.sec, (unsigned long)now_sec);
        m2mb_rtc_close(rtcfd);
        return -1;
    }

    time_val_t = (time_t)time_val.sec;
    ptm = gmtime(&time_val_t);
    if (ptm == NULL)
    {
        AZX_LOG_ERROR("gmtime() failed for epoch %lu -- cannot set alarm\r\n",
                      (unsigned long)time_val.sec);
        m2mb_rtc_close(rtcfd);
        return -1;
    }

    rtc_time.year = ptm->tm_year + 1900;
    rtc_time.mon  = ptm->tm_mon + 1;
    rtc_time.day  = ptm->tm_mday;
    rtc_time.hour = ptm->tm_hour;
    rtc_time.min  = ptm->tm_min;
    rtc_time.sec  = ptm->tm_sec;

    AZX_LOG_INFO("Arming alarm (+%lu s) at: %d-%02d-%02d, %02d:%02d:%02d\r\n",
                 (unsigned long)seconds,
                 rtc_time.year, rtc_time.mon, rtc_time.day,
                 rtc_time.hour, rtc_time.min, rtc_time.sec);

    if (m2mb_rtc_ioctl(rtcfd, M2MB_RTC_IOCTL_SET_ALARM_TIME,
                       &rtc_time, 0x01) != 0)
    {
        AZX_LOG_ERROR("RTC: SET_ALARM_TIME failed\r\n");
        m2mb_rtc_close(rtcfd);
        return -1;
    }

#if SLEEP_VERIFY_ALARM
    /* FIX [8], optional: read the alarm back out of the hardware.
     *
     * Confirm the ioctl name in your m2mb_rtc.h before enabling. Some SDK
     * versions expose M2MB_RTC_IOCTL_GET_ALARM_TIME; others do not expose a
     * read-back at all, in which case leave SLEEP_VERIFY_ALARM at 0 and
     * rely on the log line above plus a USB capture. */
    {
        M2MB_RTC_TIME_T check;
        memset(&check, 0, sizeof(check));
        if (m2mb_rtc_ioctl(rtcfd, M2MB_RTC_IOCTL_GET_ALARM_TIME, &check) != 0)
        {
            AZX_LOG_ERROR("RTC: alarm read-back failed\r\n");
            m2mb_rtc_close(rtcfd);
            return -1;
        }
        if (check.year != rtc_time.year || check.mon  != rtc_time.mon  ||
            check.day  != rtc_time.day  || check.hour != rtc_time.hour ||
            check.min  != rtc_time.min)
        {
            AZX_LOG_ERROR("RTC: alarm read-back MISMATCH "
                          "(wrote %d-%02d-%02d %02d:%02d, read %d-%02d-%02d "
                          "%02d:%02d)\r\n",
                          rtc_time.year, rtc_time.mon, rtc_time.day,
                          rtc_time.hour, rtc_time.min,
                          check.year, check.mon, check.day,
                          check.hour, check.min);
            m2mb_rtc_close(rtcfd);
            return -1;
        }
        AZX_LOG_INFO("RTC: alarm read-back OK\r\n");
    }
#endif

    m2mb_rtc_close(rtcfd);
    return 0;
}

int SET_ALARM(void)
{
    return SET_ALARM_IN((UINT32)SLEEP_INTERVAL);
}

/* ---- shutdown paths ------------------------------------------------------- */

/*
 * FIX [3]: never leave the device awake with no alarm pending.
 * Retry, then reboot as a recovery of last resort.
 *
 * NOTE ON THE TRADE-OFF: rebooting risks a reboot loop if the RTC is
 * genuinely broken, which drains the battery over hours. Returning instead
 * (the original behavior) leaves the modem awake at full power, which
 * drains it in well under an hour and never recovers. Reboot is the lesser
 * failure. FIX [6] should make this path unreachable in the no-network
 * case that actually caused it in practice.
 */
void ALARM_SHUTDOWN_IN(UINT32 seconds)
{
    int attempt;

    for (attempt = 0; attempt < ALARM_SET_RETRIES; attempt++)
    {
        if (SET_ALARM_IN(seconds) == 0)          /* FIX [7]: strict test */
        {
            g_reboot_recovery_count = 0;         /* healthy cycle */
            SHUTDOWN();
            return;              /* SHUTDOWN() does not normally return */
        }
        AZX_LOG_ERROR("SET_ALARM failed (attempt %d/%d)\r\n",
                      attempt + 1, ALARM_SET_RETRIES);
        azx_sleep_ms(1000);
    }

    /* FIX [10]: do not spin forever inside one boot */
    if (++g_reboot_recovery_count > REBOOT_RECOVERY_MAX)
    {
        AZX_LOG_ERROR("Alarm unprogrammable and reboot recovery exhausted "
                      "(%d) -- powering off. Device will need an ON/OFF# "
                      "pulse (S2 or MCU) to return.\r\n",
                      g_reboot_recovery_count);
        SHUTDOWN();
        return;
    }

    AZX_LOG_ERROR("Could not program RTC alarm; rebooting to recover "
                  "instead of staying awake (%d/%d)\r\n",
                  g_reboot_recovery_count, REBOOT_RECOVERY_MAX);
    REBOOT();
}

void ALARM_SHUTDOWN(void)
{
    ALARM_SHUTDOWN_IN((UINT32)SLEEP_INTERVAL);
}

void ALARM_SHUTDOWN_AFTER_FAILURE(void)
{
    AZX_LOG_INFO("Network cycle failed -- sleeping %d s to clear the attach "
                 "backoff before retrying\r\n", SLEEP_INTERVAL_FAIL);
    ALARM_SHUTDOWN_IN((UINT32)SLEEP_INTERVAL_FAIL);
}

/*
 * FIX [9]: this must never return to a caller that assumes the module is
 * going down. If the power API cannot be initialised, escalate rather than
 * unwind into M2MB_main and sit awake at full power.
 */
void SHUTDOWN(void)
{
    M2MB_POWER_HANDLE power_handle = NULL;

    if (M2MB_RESULT_SUCCESS == m2mb_power_init(&power_handle,
                                               (m2mb_power_ind_callback) NULL,
                                               sleep_userdata))
    {
        AZX_LOG_INFO("Power off module\r\n");
        azx_sleep_ms(200);            /* let the log flush over USB0 */
        m2mb_power_shutdown(power_handle);

        /* If shutdown somehow returns, do not fall through to the caller. */
        azx_sleep_ms(5000);
        AZX_LOG_ERROR("SHUTDOWN: m2mb_power_shutdown returned; rebooting\r\n");
    }
    else
    {
        AZX_LOG_ERROR("Cannot init power apis! Rebooting instead of staying "
                      "awake\r\n");
    }

    /* Guard against SHUTDOWN <-> REBOOT ping-pong: only escalate once. */
    if (g_reboot_recovery_count <= REBOOT_RECOVERY_MAX)
    {
        g_reboot_recovery_count++;
        REBOOT();
    }
}

void REBOOT(void)
{
    M2MB_POWER_HANDLE power_handle = NULL;

    AZX_LOG_INFO("REBOOT: rebooting module...\r\n");
    azx_sleep_ms(500);   /* let the log flush before the radio goes down */

    if (M2MB_RESULT_SUCCESS == m2mb_power_init(&power_handle,
                                               (m2mb_power_ind_callback) NULL,
                                               sleep_userdata))
    {
        m2mb_power_reboot(power_handle);
    }

    /* Should never get here. If the reboot request failed, shut down so
     * the alarm cycle recovers the device instead of hanging awake and
     * draining the battery.
     *
     * FIX [9]: SHUTDOWN() can now call back into REBOOT(). The
     * g_reboot_recovery_count guard in SHUTDOWN() is what stops that pair
     * from bouncing. */
    AZX_LOG_ERROR("REBOOT: m2mb_power_reboot failed, shutting down instead\r\n");
    SHUTDOWN();
}
