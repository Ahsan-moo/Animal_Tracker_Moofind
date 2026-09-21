/* =============================================================================
 * main.c  --  application orchestrator
 *
 * Cycle: wake -> events init -> LWM2M init -> network -> GNSS -> battery
 *        -> LWM2M report -> alarm shutdown.
 *
 * Modules:
 *   app_common.c/.h  - event group + bounded waits
 *   at_helper.c/.h   - shared AT command helper (accumulating buffer)
 *   net_fn.c/.h      - network/PDP bring-up, manual PLMN fallback,
 *                      NB-IoT DNS workaround
 *   gnss_fn.c/.h     - GNSS fix, XTRA enable, RTC-from-GNSS
 *   lwm2m_fn.c/.h    - LWM2M client control + read/write helpers
 *   batt_fn.c/.h     - battery read + LWM2M battery resource update
 *   SLEEP_FN.c/.h    - RTC alarm, shutdown, reboot
 *
 * =============================================================================
 * FIXES CARRIED FORWARD FROM THE PREVIOUS REVISION (unchanged, still correct)
 * =============================================================================
 *
 * [1] 80 s of dead awake time per cycle removed (DEBUG_BUILD gates the 20 s
 *     startup delay and the 60 s pre-shutdown grace window).
 * [2] NET_ROUTINE's return value is honoured.
 * [3] GNSS steals the radio; a re-attach wait sits between GNSS and the
 *     LWM2M report.
 * [4] ADC_FN.h include removed.
 * [5] Post-GNSS re-attach ceiling scaled to how long GNSS held the radio.
 *
 * =============================================================================
 * NEW IN THIS REVISION -- FIXES THE "WORKS ON LAPTOP USB, DEAD ON EXTERNAL
 * POWER" REGRESSION
 * =============================================================================
 *
 * [11] ROOT CAUSE: the report was being skipped on a network state that was
 *      already stale.
 *
 *      FIX [2] correctly stopped treating NET_ROUTINE's result as always
 *      TRUE. But the previous revision then used that one result as the
 *      sole gate on the whole LWM2M report:
 *
 *          net_ok = NET_ROUTINE();
 *          GNSS_ROUTINE();               <-- 40 s .. 5 min elapse HERE
 *          BATT_STAT_ROUTINE();
 *          if (net_ok) { ... LWM2M_ROUTINE(); }
 *          else        { skip entirely; }
 *
 *      GNSS_ROUTINE takes the cellular link down and can run for minutes.
 *      By the time we reach the report, `net_ok` describes the network as
 *      it was BEFORE all of that. A modem that was still attaching when
 *      NET_ROUTINE gave up is very often fully registered by the time the
 *      GNSS fix completes -- but the old gate had already decided not to
 *      look.
 *
 *      On laptop USB the attach was fast enough that NET_ROUTINE returned
 *      TRUE and the gate never fired. On external power the attach was
 *      slower (cold start, no cached PLMN), NET_ROUTINE returned FALSE,
 *      and the device deliberately skipped the portal report -- then slept
 *      SLEEP_INTERVAL_FAIL (1200 s), so nothing appeared for 20 minutes.
 *
 *      In the revision BEFORE FIX [2], NET_ROUTINE always returned TRUE,
 *      so LWM2M_ROUTINE always ran and eventually succeeded. That is why
 *      the symptom appeared only after the update. The old behaviour was
 *      wasteful but accidentally masked this.
 *
 *      FIX: ask the modem what its state is NOW, rather than trusting a
 *      result from several minutes ago. NET_WAIT_REGISTERED() is called on
 *      both paths. A failed NET_ROUTINE gets a shorter re-check window
 *      than a successful one, so a genuinely dead network still costs
 *      little, but a late attach is no longer thrown away.
 *
 * [12] The cycle now reports why it ended. A single summary line before
 *      shutdown records the outcome of every stage, so a failure can be
 *      attributed from one line instead of reconstructed from the whole
 *      log. Costs one log call per cycle.
 *
 * [13] FORCE_LWM2M_REPORT diagnostic switch (default 0). Setting it to 1
 *      makes the report run unconditionally, reproducing the pre-FIX[2]
 *      behaviour. This is the A/B test for the diagnosis above: if the
 *      portal appears with 1 and not with 0, the gate was the cause.
 *
 * [14] LOG SAFETY. azx logging is configured onto USB0 in this project
 *      (see the "let the log flush over USB0" comment in SLEEP_FN.c).
 *      With a charger or power bank, VBUS is present but no host drains
 *      the CDC endpoint. Set PRODUCTION_BUILD to 1 for any build that will
 *      run away from a laptop. See also app_cfg.h.
 *
 * [15] LOG SAFETY, CORRECTED. Revision [14] dropped the log level to NONE
 *      on the line AFTER AZX_LOG_INIT(). That was one line too late: the
 *      init call is what opens the USB0 channel, so if the open stalls,
 *      the setLevel() never runs. PRODUCTION_BUILD now skips the init
 *      entirely. See the long note at the call site in M2MB_main().
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "m2mb_types.h"
#include "m2mb_os_api.h"

#include "app_cfg.h"
#include "azx_log.h"
#include "azx_utils.h"

#include "app_common.h"
#include "at_helper.h"
#include "net_fn.h"
#include "gnss_fn.h"
#include "lwm2m_fn.h"
#include "batt_fn.h"

#include "sleep_fn.h"   /* ALARM_SHUTDOWN(), ALARM_SHUTDOWN_AFTER_FAILURE(),
                         * REBOOT() */

/* =============================================================================
 * BUILD CONFIGURATION  --  the only things you should need to change
 * ========================================================================== */

/* ---------------------------------------------------------------------------
 * DEBUG_BUILD
 *   1 = bench work with a laptop attached. Adds a 20 s startup window so a
 *       terminal can attach, and a 60 s grace window before shutdown.
 *   0 = field/production. Costs 80 s of full-power awake time per cycle if
 *       left at 1, so it must be 0 for anything battery powered.
 * ------------------------------------------------------------------------- */
#ifndef DEBUG_BUILD
#define DEBUG_BUILD              0
#endif

/* ---------------------------------------------------------------------------
 * PRODUCTION_BUILD                                                  FIX [14]
 *   1 = silence azx logging immediately after init. REQUIRED for any build
 *       that will run from a charger, power bank, lab supply or battery,
 *       because the log channel is USB0 and a charger presents VBUS with no
 *       host reading the endpoint.
 *   0 = keep logging (laptop-attached bench work only).
 *
 *   Set this to 1 and DEBUG_BUILD to 0 for field images.
 * ------------------------------------------------------------------------- */
#ifndef PRODUCTION_BUILD
#define PRODUCTION_BUILD         1
#endif

/* ---------------------------------------------------------------------------
 * FORCE_LWM2M_REPORT                                                FIX [13]
 *   0 = normal. The report runs only when the modem is confirmed registered.
 *   1 = DIAGNOSTIC ONLY. Runs the report regardless of network state, which
 *       reproduces the behaviour of the revision before FIX [2].
 *
 *   Use: flash with 1, run on external power, wait 25 minutes, check the
 *   portal. If the device appears with 1 but not with 0, the registration
 *   gate is confirmed as the cause and the re-check window below should be
 *   widened rather than the gate removed.
 *
 *   Do not ship with this at 1: a cycle with no network will burn the whole
 *   LWM2M budget failing to reach the server.
 * ------------------------------------------------------------------------- */
#ifndef FORCE_LWM2M_REPORT
#define FORCE_LWM2M_REPORT       0
#endif

#define DEBUG_STARTUP_DELAY_MS   20000   /* terminal attach window  */
#define DEBUG_GRACE_DELAY_MS     60000   /* pre-shutdown log window */

/* ---------------------------------------------------------------------------
 * Re-attach / re-check windows                                      FIX [11]
 *
 * REATTACH_WAIT_MIN_MS / MAX_MS
 *   Used when NET_ROUTINE succeeded. GNSS then held the radio for somewhere
 *   between ~40 s (warm fix) and the full GNSS budget (timeout), so the
 *   ceiling is scaled by GNSS_LAST_TIMED_OUT().
 *
 * RECHECK_AFTER_NET_FAIL_MS
 *   Used when NET_ROUTINE FAILED. This is the new path. The modem may have
 *   completed its attach during GNSS, so we ask rather than assume.
 *
 *   90 s is chosen as follows: a modem that was mid-attach when NET_ROUTINE
 *   gave up will normally complete within one T3412 periodic-update cycle
 *   of being released by GNSS. A modem that is genuinely not on the network
 *   will not recover in any window, so a longer wait only wastes power.
 *   If your coverage is marginal and the A/B test in FIX [13] shows the
 *   portal appearing only with FORCE_LWM2M_REPORT=1, raise this value
 *   rather than removing the gate.
 * ------------------------------------------------------------------------- */
#define REATTACH_WAIT_MIN_MS       30000UL   /* after a short GNSS cycle */
#define REATTACH_WAIT_MAX_MS       90000UL   /* after a GNSS timeout     */
#define RECHECK_AFTER_NET_FAIL_MS  90000UL   /* after NET_ROUTINE failed */

/* Set to 1 to run the LWM2M set/read scratch test instead of skipping it */
#define ENABLE_LWM2M_TEST        0

#if ENABLE_LWM2M_TEST
static BOOLEAN test(void)
{
    AZX_LOG_INFO("starting test\r\n");
    char cmd_str[128];
    char resp_str[256];
    int adc_read = 0;

    if (!ati_send_and_wait("AT#LWM2MR=0,3,0,9,0\r", resp_str,
                           sizeof(resp_str), 5000))
    {
        AZX_LOG_INFO("AT#LWM2MR=0,3,0,9,0 failed\r\n");
        return FALSE;
    }

    /* NULL-check before dereferencing strstr result */
    char *p = strstr(resp_str, "#LWM2MR:");
    if (p == NULL)
    {
        AZX_LOG_ERROR("#LWM2MR tag not found\r\n");
        return FALSE;
    }
    adc_read = atoi(p + 8);
    AZX_LOG_INFO("value:%d\r\n", adc_read);

    adc_read += 5;
    AZX_LOG_INFO("Writing new value: %d\r\n", adc_read);

    snprintf(cmd_str, sizeof(cmd_str), "AT#LWM2MSET=0,3,0,9,0,%d\r", adc_read);
    if (!ati_send_and_wait(cmd_str, resp_str, sizeof(resp_str), 5000))
    {
        AZX_LOG_INFO("AT#LWM2MSET failed for value %d\r\n", adc_read);
        return FALSE;
    }

    LWM2M_ROUTINE();
    AZX_LOG_INFO("Rebooting...\r\n");
    REBOOT();
    return TRUE;
}
#endif /* ENABLE_LWM2M_TEST */


void M2MB_main(int argc, char **argv)
{
    /* ---- per-cycle outcome flags, for the summary in FIX [12] ---------- */
    BOOLEAN net_ok       = FALSE;   /* NET_ROUTINE result                  */
    BOOLEAN registered   = FALSE;   /* modem confirmed on-network at report*/
    BOOLEAN gnss_ok      = FALSE;   /* GNSS fix this cycle                 */
    BOOLEAN batt_ok      = FALSE;   /* battery resource updated            */
    BOOLEAN reported     = FALSE;   /* LWM2M registration observed         */
    const char *end_reason = "unknown";

    (void)argc;
    (void)argv;

#if DEBUG_BUILD
    /* bench only -- compiled out of production images */
    azx_sleep_ms(DEBUG_STARTUP_DELAY_MS);
#endif

    /* -----------------------------------------------------------------------
     * FIX [15]: DO NOT OPEN THE LOG CHANNEL AT ALL IN A PRODUCTION BUILD.
     *
     * The previous revision called AZX_LOG_INIT() unconditionally and then
     * dropped the level to NONE on the next line. That was one line too
     * late. AZX_LOG_INIT() expands to azx_log_init(&cfg), which OPENS the
     * configured channel -- USB0 in this project. If that open call itself
     * stalls, execution never reaches azx_log_setLevel() and the
     * application is dead before NET_ROUTINE() is ever called.
     *
     * The stall condition is specific and is exactly the field case:
     *
     *   Laptop, terminal open : VBUS present, host enumerates and drains
     *                           the CDC endpoint. Open succeeds, logs flow.
     *
     *   Charger / power bank  : VBUS present, NO enumeration, nothing
     *                           drains the endpoint. azx_log does not
     *                           report AZX_LOG_USB_CABLE_UNPLUGGED because
     *                           from the module's point of view the cable
     *                           IS plugged.
     *
     *   Lab supply / battery  : no VBUS. The unplugged case is detected
     *                           and handled.
     *
     * Skipping the init entirely removes the failure mode rather than
     * trying to outrun it. Every AZX_LOG_* call elsewhere in the project
     * stays safe: azx_log_formatted() returns AZX_LOG_NOT_INIT and does
     * nothing when the log system was never initialised (see the
     * AZX_LOG_ERRORS_E enum in azx_log.h).
     *
     * This is deliberately done here rather than with -DAZX_LOG_DISABLE in
     * the build settings. That macro must be visible in EVERY translation
     * unit to take effect, and several files in this project (gnss_fn.c
     * among them) include azx_log.h directly without including app_cfg.h
     * first -- so a header-based define would silence some files and not
     * others. Guarding the single init call site has no such dependency.
     *
     * If you would rather use the build flag as well, add
     * -DAZX_LOG_DISABLE to the project defines. It is complementary, not
     * a replacement, and it also compiles out the call sites themselves.
     * -------------------------------------------------------------------- */
#if PRODUCTION_BUILD
    /* Channel never opened. azx_log_setLevel() is not called either: with
     * no init there is no log system to set a level on. */
#else
    AZX_LOG_INIT();
#endif

    AZX_LOG_INFO("STARTING APPLICATION!!!\r\n");

    /* -- 1. Event group -- */
    if (!app_ev_init())
    {
        AZX_LOG_ERROR("event init failed, shutting down to preserve battery\r\n");
        ALARM_SHUTDOWN();
        return;
    }

    /* -- 2. LWM2M client init -- */
    if (!LWM2M_initializer())
    {
        AZX_LOG_ERROR("LWM2M init failed, continuing (GNSS storage and "
                      "registration will be skipped internally)\r\n");
    }

#if ENABLE_LWM2M_TEST
    if (test())
        AZX_LOG_INFO("test successful\r\n");
    else
        AZX_LOG_INFO("test failed\r\n");
#endif

    /* -- 3. Network + PDP -- */
    net_ok = NET_ROUTINE();
    if (!net_ok)
    {
        /* FIX [11]: this is NOT final any more. GNSS runs next and takes
         * minutes; the modem may well attach during it. We re-check below
         * instead of writing the cycle off here. */
        AZX_LOG_ERROR("NET_ROUTINE failed: no usable connectivity yet "
                      "(will re-check after GNSS)\r\n");
    }

    /* -- 4. GNSS fix + XTRA (does not need the network) -- */
    gnss_ok = GNSS_ROUTINE();
    if (!gnss_ok)
    {
        AZX_LOG_INFO("GNSS_ROUTINE: no fix this cycle\r\n");
    }

    /* -- 5. Battery -> LWM2M resource (local write, no network needed) -- */
    batt_ok = BATT_STAT_ROUTINE();
    if (!batt_ok)
    {
        AZX_LOG_INFO("BATT_STAT_ROUTINE failed\r\n");
    }

    /* -----------------------------------------------------------------------
     * -- 6. LWM2M registration + report window --                   FIX [11]
     *
     * Ask the modem for its CURRENT registration state rather than trusting
     * net_ok, which was measured before GNSS held the radio for up to five
     * minutes. Both paths get a bounded wait; the failed path gets a
     * shorter one so a genuinely dead network stays cheap.
     * -------------------------------------------------------------------- */
    {
        UINT32 wait_ms;

        if (net_ok)
        {
            /* Scale to how long GNSS actually held the radio. */
            wait_ms = GNSS_LAST_TIMED_OUT() ? REATTACH_WAIT_MAX_MS
                                            : REATTACH_WAIT_MIN_MS;

            AZX_LOG_INFO("GNSS held the radio ~%lu s; allowing up to %lu s "
                         "to re-attach\r\n",
                         (unsigned long)(GNSS_LAST_HELD_RADIO_MS() / 1000UL),
                         (unsigned long)(wait_ms / 1000UL));
        }
        else
        {
            /* NEW PATH. NET_ROUTINE failed earlier in this cycle, but that
             * was before GNSS. Give the modem a bounded chance to show that
             * it attached in the meantime. */
            wait_ms = RECHECK_AFTER_NET_FAIL_MS;

            AZX_LOG_INFO("Network was down before GNSS; re-checking for up "
                         "to %lu s in case it attached since\r\n",
                         (unsigned long)(wait_ms / 1000UL));
        }

        registered = NET_WAIT_REGISTERED(wait_ms);

        if (registered && !net_ok)
        {
            /* Worth calling out: this is exactly the cycle the previous
             * revision threw away. */
            AZX_LOG_INFO("Modem attached during GNSS -- reporting after all\r\n");
        }
    }

#if FORCE_LWM2M_REPORT
    /* FIX [13]: diagnostic override. Reproduces pre-FIX[2] behaviour. */
    if (!registered)
    {
        AZX_LOG_ERROR("FORCE_LWM2M_REPORT=1: running the report even though "
                      "the modem is NOT registered. Diagnostic build only.\r\n");
    }
    registered = TRUE;
#endif

    if (registered)
    {
        reported = LWM2M_ROUTINE();
        if (!reported)
        {
            AZX_LOG_INFO("LWM2M_ROUTINE: client did not register this cycle\r\n");
            end_reason = "lwm2m_no_registration";
        }
        else
        {
            end_reason = "ok";
        }
    }
    else
    {
        /* Starting the LWM2M client here would only burn its whole budget
         * failing to reach the server. */
        AZX_LOG_ERROR("Modem not registered after re-check; skipping LWM2M "
                      "report this cycle\r\n");
        end_reason = "not_registered";
    }

#if DEBUG_BUILD
    /* bench only -- compiled out of production images */
    azx_sleep_ms(DEBUG_GRACE_DELAY_MS);
#endif

    /* -----------------------------------------------------------------------
     * -- 7. Cycle summary --                                        FIX [12]
     *
     * One line that says what happened. When a unit misbehaves in the field
     * this is the line to ask for; it removes the need to reconstruct the
     * cycle from the whole log.
     * -------------------------------------------------------------------- */
    AZX_LOG_INFO("CYCLE SUMMARY: net=%d reg=%d gnss=%d batt=%d report=%d "
                 "reason=%s\r\n",
                 (int)net_ok, (int)registered, (int)gnss_ok,
                 (int)batt_ok, (int)reported, end_reason);

    /* -----------------------------------------------------------------------
     * -- 8. Sleep until the next alarm --
     *
     * The sleep length is chosen on whether the REPORT succeeded, not on
     * net_ok. A cycle that recovered late and reported successfully is a
     * healthy cycle and takes the normal cadence.
     * -------------------------------------------------------------------- */
    if (reported)
    {
        ALARM_SHUTDOWN();
    }
    else
    {
        /* Sleep past the network's attach backoff so the next cold start is
         * not rejected on arrival. */
        ALARM_SHUTDOWN_AFTER_FAILURE();
    }
}
