/* =============================================================================
 * gnss_fn.c
 *
 * FIXES vs original main.c:
 *  - GNSS_ROUTINE was declared BOOLEAN but never returned a value
 *    (undefined behavior). It now returns the fix result.
 *  - obj URIs use fresh C aggregate initializers instead of C++-style
 *    brace re-assignment.
 *  - The 10-byte m2mb_os_malloc (unchecked, unaligned, leaked) is
 *    replaced with a static, properly aligned UINT32 buffer.
 *  - LWM2M reads/writes go through lwm2m_fn helpers with bounded waits
 *    on dedicated event bits (old code waited FOREVER on the shared bit).
 *  - GNSS init retry bound kept (10 attempts), priority-switch loops are
 *    now bounded too so a modem wedge can't hang the wake cycle.
 *  - XTRA helpers use the shared accumulating AT helper.
 *
 * XTRA notes (unchanged intent from original):
 *  - m2mb_gnss.h in this SDK does not expose m2mb_gnss_set_agnss_enable(),
 *    so AT$AGNSS is used via ATI.
 *  - XTRA must only be enabled AFTER a valid fix; the setting persists in
 *    NVM and takes effect on the next power-on, which the normal
 *    alarm/shutdown cycle provides. No mid-cycle reboot.
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

#define GNSS_FIX_POLL_MAX        30       /* x 10 s  -> up to 5 min       */
#define GNSS_FIX_POLL_PERIOD_MS  10000
#define RTC_SANE_EPOCH           1704067200 /* 2024-01-01: RTC older than
                                             * this was never network-set */

static M2MB_GNSS_HANDLE gnss_handle = NULL;

/* ---- GNSS callback (position/NMEA indications, if the FW emits them) ------ */
static void gnss_callback(M2MB_GNSS_HANDLE h, M2MB_GNSS_IND_E gnss_event,
                          UINT16 resp_size, void *resp, void *userdata)
{
    (void)h; (void)resp_size; (void)userdata;

    switch (gnss_event)
    {
    case M2MB_GNSS_INDICATION_POSITION_REPORT:
    {
        /* FIX: read fields inside the callback only; do not store the
         * pointer for later use */
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
 * Must only be called AFTER a successful GNSS fix. */
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

/* ---- main GNSS routine ---------------------------------------------------- */
BOOLEAN GNSS_ROUTINE(void)
{
    int ctr;
    BOOLEAN gnss_fix_obtained = FALSE;

    /* FIX: static aligned buffer instead of unchecked 10-byte malloc leak */
    static UINT32 obj_out[4];

    /* -- 1. Init GNSS -- */
    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_gnss_init(&gnss_handle,
                                                 gnss_callback, NULL))
    {
        AZX_LOG_INFO("m2mb_gnss_init failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 10)
            return FALSE;
    }
    AZX_LOG_INFO("m2mb_gnss_init succeeded\r\n");

    /* -- 2. Check XTRA status (log only; enable happens after first fix) -- */
    BOOLEAN xtra_already_active = is_xtra_active();
    if (xtra_already_active)
        AZX_LOG_INFO("XTRA is active -- warm/hot start expected\r\n");
    else
        AZX_LOG_INFO("XTRA not yet active -- cold start this cycle, "
                     "will enable after first fix\r\n");

    /* -- 3. Switch modem to GNSS priority (FIX: bounded) -- */
    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_gnss_set_prio_runtime(gnss_handle,
                                                             GNSS_PRIORITY))
    {
        AZX_LOG_INFO("m2mb_gnss_set_prio_runtime_gnss failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 30)
        {
            AZX_LOG_ERROR("Could not switch to GNSS priority\r\n");
            return FALSE;
        }
    }
    AZX_LOG_INFO("m2mb_gnss_set_prio_runtime_gnss succeeded\r\n");

    /* -- 4. Object 33211 enables location data storage at object 6 -- */
    {
        M2MB_LWM2M_OBJ_URI_T uri_storage = { 4, 33211, 0, 0, 0 };
        if (!lwm2m_write_u32(&uri_storage, 1))
            AZX_LOG_ERROR("Failed to enable location storage (33211)\r\n");
    }

    /* -- 5. Poll /6/0/5 (timestamp) for a fix -- */
    ctr = 0;
    while (ctr < GNSS_FIX_POLL_MAX)
    {
        M2MB_LWM2M_OBJ_URI_T uri_time = { 4, 6, 0, 5, 0 };

        obj_out[0] = 0;
        if (lwm2m_read_raw(&uri_time, obj_out, sizeof(obj_out), 15000))
        {
            AZX_LOG_INFO("fix timestamp: %lu\r\n", (unsigned long)obj_out[0]);
            if (obj_out[0] > 0)
            {
                gnss_fix_obtained = TRUE;
                maybe_set_rtc_from_gnss(obj_out[0]);
                break;
            }
        }
        else
        {
            AZX_LOG_INFO("lwm2m read of /6/0/5 timed out/failed\r\n");
        }

        azx_sleep_ms(GNSS_FIX_POLL_PERIOD_MS);
        ctr++;
    }

    /* -- 6. After fix: enable XTRA if not already active -- */
    if (gnss_fix_obtained && !xtra_already_active)
    {
        request_xtra_enable();
    }

    /* -- 7. Disable location storage, restore WWAN priority -- */
    {
        M2MB_LWM2M_OBJ_URI_T uri_storage = { 4, 33211, 0, 0, 0 };
        lwm2m_write_u32(&uri_storage, 0);
    }

    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_gnss_set_prio_runtime(gnss_handle,
                                                             WWAN_PRIORITY))
    {
        AZX_LOG_INFO("m2mb_gnss_set_prio_runtime_wwan failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 30)
        {
            AZX_LOG_ERROR("Could not restore WWAN priority\r\n");
            break;   /* still return the fix result */
        }
    }
    if (ctr <= 30)
        AZX_LOG_INFO("m2mb_gnss_set_prio_runtime_wwan succeeded\r\n");

    return gnss_fix_obtained;   /* FIX: function actually returns now */
}
