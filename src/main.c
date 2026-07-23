/* =============================================================================
 * main.c  --  application orchestrator
 *
 * Cycle: wake -> events init -> LWM2M init -> network -> GNSS -> battery
 *        -> LWM2M report -> alarm shutdown.
 *
 * All functionality now lives in dedicated modules:
 *   app_common.c/.h  - event group + bounded waits
 *   at_helper.c/.h   - shared AT command helper (accumulating buffer)
 *   net_fn.c/.h      - network/PDP bring-up, manual PLMN fallback,
 *                      NB-IoT DNS workaround
 *   gnss_fn.c/.h     - GNSS fix, XTRA enable, RTC-from-GNSS
 *   lwm2m_fn.c/.h    - LWM2M client control + read/write helpers
 *   batt_fn.c/.h     - battery read + LWM2M battery resource update
 *
 * Behavior is intentionally unchanged: even if the network routine fails,
 * the remaining stages still run (as in the original), each stage guards
 * itself, and the device always reaches ALARM_SHUTDOWN() at the end.
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

#include "ADC_FN.h"
#include "SLEEP_FN.h"   /* ALARM_SHUTDOWN(), REBOOT() */

/* Set to 1 to run the LWM2M set/read scratch test instead of skipping it */
#define ENABLE_LWM2M_TEST  0

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

    /* FIX: NULL-check before dereferencing strstr result */
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
    (void)argc;
    (void)argv;

    /* Startup delay so a debug terminal can attach before the first logs.
     * NOTE: this costs 20 s of awake time on EVERY cycle -- consider
     * gating it behind a debug build flag for production. */
    azx_sleep_ms(20000);
    AZX_LOG_INIT();
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
    if (!NET_ROUTINE())
    {
        AZX_LOG_ERROR("NET_ROUTINE failed (continuing cycle as before)\r\n");
    }

    /* -- 4. GNSS fix + XTRA -- */
    if (!GNSS_ROUTINE())
    {
        AZX_LOG_INFO("GNSS_ROUTINE: no fix this cycle\r\n");
    }

    /* -- 5. Battery -> LWM2M resource -- */
    if (!BATT_STAT_ROUTINE())
    {
        AZX_LOG_INFO("BATT_STAT_ROUTINE failed\r\n");
    }

    /* -- 6. LWM2M registration + report window -- */
    if (!LWM2M_ROUTINE())
    {
        AZX_LOG_INFO("LWM2M_ROUTINE: client did not register this cycle\r\n");
    }

    /* -- 7. Grace period, then sleep until next alarm -- */
    azx_sleep_ms(60000);
    ALARM_SHUTDOWN();
}
