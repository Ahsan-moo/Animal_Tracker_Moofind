/* =============================================================================
 * batt_fn.c
 *
 * =============================================================================
 * FIXES IN THIS REVISION
 * =============================================================================
 *
 * [1] THE UNIT AUTO-DETECT THRESHOLD WAS WRONG, so the battery always
 *     reported 100%.
 *
 *     The previous code was:
 *         if (battery_raw > 6000) battery_mV = battery_raw;
 *         else                    battery_mV = battery_raw * 10;
 *
 *     A 1S LiPo reads 3200-4200 in mV, or 320-420 in tens-of-mV. BOTH
 *     representations are below 6000, so the else branch always ran. On
 *     firmware that reports plain mV, 3800 became 38000 mV and the
 *     percentage clamped to 100 permanently. The file's own header claimed
 *     this was fixed; it was not.
 *
 *     Threshold is now 1000, which sits cleanly between the two ranges.
 *
 * [2] BATT_MV_100 was 3700, so anything above 3.7 V read as full. A 1S LiPo
 *     runs to 4.2 V, so roughly the top third of the charge curve was
 *     invisible. Now 4200.
 *
 *     NOTE: this CHANGES REPORTED VALUES vs the old firmware. A pack that
 *     used to report 100% at 3.75 V will now report about 55%. If a
 *     back-end has alert thresholds tuned against the old curve, retune
 *     them or keep BATT_MV_100 at 3700 deliberately.
 *
 * [3] Raw value is now sanity-checked before use. A malformed #CBC reply
 *     previously flowed straight into the percentage calculation.
 *
 * Preserved: snprintf over sprintf, correct format specifiers, shared
 * accumulating AT helper.
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "m2mb_types.h"

#include "azx_log.h"

#include "at_helper.h"
#include "batt_fn.h"

/* ---- LiPo mapping ---------------------------------------------------------
 * FIX [2]: full scale is 4200 mV, not 3700 mV. */
#define BATT_MV_0     3200
#define BATT_MV_100   4200

/* FIX [1]: below this, the reading is tens-of-mV; at or above, plain mV.
 * 1S LiPo: 3200-4200 mV, or 320-420 tens-of-mV. 1000 separates them. */
#define BATT_UNIT_THRESHOLD  1000

/* Plausibility window for the converted value. Anything outside this is a
 * parse failure, not a real battery. */
#define BATT_MV_MIN   2000
#define BATT_MV_MAX   6000

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

    if (battery_raw <= 0)
    {
        AZX_LOG_ERROR("#CBC returned a non-positive value (%d)\r\n", battery_raw);
        return FALSE;
    }

    /* FIX [1]: correct unit auto-detect (tens-of-mV vs plain mV, FW-dependent) */
    if (battery_raw >= BATT_UNIT_THRESHOLD)
        battery_mV = battery_raw;         /* already mV      */
    else
        battery_mV = battery_raw * 10;    /* tens of mV      */

    AZX_LOG_INFO("#CBC raw:%d -> %d mV\r\n", battery_raw, battery_mV);

    /* FIX [3]: reject implausible conversions instead of reporting them */
    if (battery_mV < BATT_MV_MIN || battery_mV > BATT_MV_MAX)
    {
        AZX_LOG_ERROR("Battery voltage %d mV out of plausible range "
                      "(%d-%d), treating as a parse failure\r\n",
                      battery_mV, BATT_MV_MIN, BATT_MV_MAX);
        return FALSE;
    }

    /* -- 2. Battery level -- */
    {
        float percent = ((float)(battery_mV - BATT_MV_0) /
                         (float)(BATT_MV_100 - BATT_MV_0)) * 100.0f;
        if (percent < 0.0f)   percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        level_pct = (int)percent;
    }
    AZX_LOG_INFO("Battery: %d mV, level: %d%%\r\n", battery_mV, level_pct);

    /* -- 3. Update battery level resource /3/0/9 for the m2m agent -- */
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
