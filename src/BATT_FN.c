/* =============================================================================
 * batt_fn.c
 *
 * FIXES vs original main.c:
 *  - #CBC unit auto-detect: if the reported value is already in mV
 *    (> 6000), it is used as-is; otherwise it is treated as tens-of-mV
 *    and multiplied by 10 (the old code unconditionally multiplied,
 *    which clamps percent to 100% forever if the FW reports plain mV).
 *  - snprintf instead of sprintf for command building.
 *  - Format-string bugs removed (old code passed adc_read to log calls
 *    whose format string had no specifier).
 *  - Uses the shared accumulating AT helper.
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "m2mb_types.h"

#include "azx_log.h"

#include "at_helper.h"
#include "batt_fn.h"

/* LiPo mapping used by the original code: 3200 mV = 0%, 3700 mV = 100%.
 * NOTE: a 1S LiPo actually spans ~3200-4200 mV; everything above 3700
 * will read 100%. Kept as-is per current product behavior -- widen
 * BATT_MV_100 to 4200 if you want a real fuel gauge across full charge. */
#define BATT_MV_0     3200
#define BATT_MV_100   3700

BOOLEAN BATT_STAT_ROUTINE(void)
{
    char cmd_str[128];
    char resp_str[256];
    int battery_raw = 0;
    int battery_mV  = 0;
    int level_pct   = 0;

    /* -- 1. Read battery voltage via AT#CBC -- */
    if (!ati_send_and_wait("AT#CBC\r", resp_str, sizeof(resp_str), 5000))
    {
        AZX_LOG_INFO("AT#CBC failed\r\n");
        return FALSE;
    }

    {
        char *p = strstr(resp_str, "#CBC:");
        if (p == NULL)
        {
            AZX_LOG_ERROR("#CBC tag not found in response\r\n");
            return FALSE;
        }
        char *comma = strchr(p, ',');
        if (comma == NULL)
        {
            AZX_LOG_ERROR("#CBC response malformed\r\n");
            return FALSE;
        }
        battery_raw = atoi(comma + 1);
    }

    /* FIX: unit auto-detect (tens-of-mV vs plain mV, FW-dependent) */
    if (battery_raw > 6000)
        battery_mV = battery_raw;         /* already mV */
    else
        battery_mV = battery_raw * 10;    /* tens of mV */

    AZX_LOG_INFO("#CBC raw:%d -> %d mV\r\n", battery_raw, battery_mV);

    /* -- 2. Battery level -- */
    {
        float percent = ((float)(battery_mV - BATT_MV_0) /
                         (float)(BATT_MV_100 - BATT_MV_0)) * 100.0f;
        if (percent < 0.0f)   percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        level_pct = (int)percent;
    }
    AZX_LOG_INFO("Battery: %d mV, level: %d%%\r\n", battery_mV, level_pct);

    /* -- 3. Update battery status object /3/0/9 for the m2m agent -- */
    snprintf(cmd_str, sizeof(cmd_str), "AT#LWM2MSET=0,3,0,9,0,%d\r", level_pct);
    if (!ati_send_and_wait(cmd_str, resp_str, sizeof(resp_str), 5000))
    {
        AZX_LOG_INFO("AT#LWM2MSET=0,3,0,9,0,%d failed\r\n", level_pct);
        return FALSE;
    }

    /* Read back for verification */
    if (!ati_send_and_wait("AT#LWM2MR=0,3,0,9,0\r", resp_str,
                           sizeof(resp_str), 5000))
    {
        AZX_LOG_INFO("AT#LWM2MR=0,3,0,9,0 failed\r\n");
        return FALSE;
    }

    return TRUE;
}
