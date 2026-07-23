/* =============================================================================
 * lwm2m_fn.c
 *
 * FIXES vs original main.c:
 *  - `if(30)` / `if(300)` in the retry loops were ALWAYS TRUE, so both
 *    loops ran exactly one iteration, broke out, and then logged
 *    "successful" regardless. Enable now really retries up to 30x and the
 *    registration wait really polls up to ~300 s.
 *  - Callback stat result is deep-copied into a module-owned struct
 *    (old code kept the raw resp_struct pointer).
 *  - M2MB_LWM2M_ENABLE_REQ_T / obj URIs use plain C aggregate
 *    initializers at declaration (old code used C++-style brace
 *    assignment `obj_uri = {...}` and `en_params{...}`, which is not C).
 *  - LWM2M waits use dedicated event bits with bounded timeouts instead
 *    of the shared EV_MOO_BIT + WAIT_FOREVER.
 *  - Success/failure is logged truthfully and returned to the caller.
 * ========================================================================== */
#include <string.h>

#include "m2mb_types.h"
#include "m2mb_os_api.h"
#include "m2mb_lwm2m.h"

#include "azx_log.h"
#include "azx_utils.h"

#include "app_common.h"
#include "lwm2m_fn.h"

static M2MB_LWM2M_HANDLE lwm2m_handle = NULL;

/* FIX: owned copy of the stat response, not a pointer into SDK memory */
static M2MB_LWM2M_GET_STAT_RES_T g_lwm2m_stat;
static BOOLEAN g_lwm2m_stat_valid = FALSE;

/* ---- callback ------------------------------------------------------------- */
static void LWM2MB_callback(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_EVENT_E event,
                            UINT16 resp_size, void *resp_struct, void *userdata)
{
    (void)h; (void)resp_size; (void)userdata;

    switch (event)
    {
    case M2MB_LWM2M_GET_STAT_RES:
    {
        M2MB_LWM2M_GET_STAT_RES_T *resp = (M2MB_LWM2M_GET_STAT_RES_T *)resp_struct;
        memcpy(&g_lwm2m_stat, resp, sizeof(g_lwm2m_stat));   /* FIX: copy */
        g_lwm2m_stat_valid = TRUE;
        AZX_LOG_INFO("lwm2m status: %d %d result:%d\r\n",
                     g_lwm2m_stat.status, g_lwm2m_stat.clStatus,
                     g_lwm2m_stat.result);
        set_ev(EV_LWM2M_STAT_BIT);
        break;
    }
    case M2MB_LWM2M_READ_RES:
    {
        M2MB_LWM2M_READ_RES_T *resp = (M2MB_LWM2M_READ_RES_T *)resp_struct;
        AZX_LOG_INFO("lwm2m read: datatype:%d length:%d\r\n",
                     resp->resType, resp->len);
        set_ev(EV_LWM2M_READ_BIT);
        break;
    }
    case M2MB_LWM2M_WRITE_RES:
    {
        /* FIX: was commented out; now signaled so writes can be confirmed */
        set_ev(EV_LWM2M_WRITE_BIT);
        break;
    }
    default:
        break;
    }
}

/* ---- init ----------------------------------------------------------------- */
BOOLEAN LWM2M_initializer(void)
{
    int ctr = 0;

    while (M2MB_RESULT_SUCCESS != m2mb_lwm2m_init(&lwm2m_handle,
                                                  LWM2MB_callback, NULL))
    {
        AZX_LOG_ERROR("m2mb_lwm2m_init failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 5)
            return FALSE;
    }
    AZX_LOG_INFO("LWM2MB initialized successfully\r\n");
    return TRUE;
}

/* ---- helpers used by GNSS module ------------------------------------------ */
BOOLEAN lwm2m_write_u32(M2MB_LWM2M_OBJ_URI_T *uri, UINT32 value)
{
    if (lwm2m_handle == NULL || uri == NULL)
        return FALSE;

    if (M2MB_RESULT_SUCCESS != m2mb_lwm2m_write(lwm2m_handle, uri, &value, 1))
    {
        AZX_LOG_ERROR("m2mb_lwm2m_write request failed\r\n");
        return FALSE;
    }
    /* Bounded wait for the write ack; non-fatal if it never arrives
     * (some agent versions do not emit WRITE_RES for every write). */
    if (!wait_ev(EV_LWM2M_WRITE_BIT, 5000))
    {
        AZX_LOG_INFO("lwm2m write: no WRITE_RES within 5s (continuing)\r\n");
    }
    return TRUE;
}

BOOLEAN lwm2m_read_raw(M2MB_LWM2M_OBJ_URI_T *uri, void *out_buf,
                       UINT16 out_len, UINT32 timeout_ms)
{
    if (lwm2m_handle == NULL || uri == NULL || out_buf == NULL)
        return FALSE;

    if (M2MB_RESULT_SUCCESS != m2mb_lwm2m_read(lwm2m_handle, uri,
                                               out_buf, out_len))
    {
        AZX_LOG_ERROR("m2mb_lwm2m_read request failed\r\n");
        return FALSE;
    }
    return wait_ev(EV_LWM2M_READ_BIT, timeout_ms);
}

/* ---- enable / registration / disable cycle -------------------------------- */
BOOLEAN LWM2M_ROUTINE(void)
{
    int ctr;
    BOOLEAN registered = FALSE;

    if (lwm2m_handle == NULL)
    {
        AZX_LOG_ERROR("LWM2M not initialized, skipping routine\r\n");
        return FALSE;
    }

    /* FIX: plain C aggregate initializer at declaration (old code used
     * C++ brace-init on an already-declared variable) */
    M2MB_LWM2M_ENABLE_REQ_T en_params = { M2MB_LWM2M_MODE_NO_ACK, 1, 5, 5,
                                          M2MB_LWM2MENA_CMD_TYPE_SET };

    /* -- 1. Enable client (FIX: really retries; old `if(30)` broke at once) */
    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_lwm2m_enable(lwm2m_handle, &en_params))
    {
        AZX_LOG_INFO("m2mb_lwm2m_enable failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 30)
        {
            AZX_LOG_ERROR("m2mb_lwm2m_enable gave up after %d attempts\r\n", ctr);
            return FALSE;
        }
    }
    AZX_LOG_INFO("m2mb_lwm2m_enable successful\r\n");

    azx_sleep_ms(1000);

    /* -- 2. Wait for client registration (clStatus == 4)
     *       (FIX: really polls up to ~300 s; old `if(300)` broke at once) */
    ctr = 0;
    while (!registered)
    {
        g_lwm2m_stat_valid = FALSE;
        if (M2MB_RESULT_SUCCESS == m2mb_lwm2m_get_stat(lwm2m_handle))
        {
            if (wait_ev(EV_LWM2M_STAT_BIT, 5000) && g_lwm2m_stat_valid)
            {
                if (g_lwm2m_stat.clStatus == 4)
                {
                    registered = TRUE;
                    break;
                }
            }
        }
        AZX_LOG_INFO("lwm2m not registered yet (clStatus=%d)\r\n",
                     g_lwm2m_stat_valid ? g_lwm2m_stat.clStatus : -1);
        azx_sleep_ms(1000);
        if (++ctr > 300)
        {
            AZX_LOG_ERROR("lwm2m registration wait timed out\r\n");
            break;
        }
    }

    if (registered)
    {
        AZX_LOG_INFO("lwm2m client registered (clStatus=4)\r\n");
        /* Give the agent time to push its reports before disabling */
        azx_sleep_ms(30000);
    }

    if (M2MB_RESULT_SUCCESS == m2mb_lwm2m_disable(lwm2m_handle))
    {
        AZX_LOG_INFO("LWM2MB disabled\r\n");
    }

    return registered;
}
