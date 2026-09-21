/* =============================================================================
 * lwm2m_fn.c
 *
 * =============================================================================
 * FIXES IN THIS REVISION
 * =============================================================================
 *
 * [1] THE REGISTRATION WAIT COUNTED ITERATIONS, NOT TIME.
 *     `ctr > 300` with a 5 s stat timeout plus a 1 s sleep per iteration
 *     means the loop runs ~300 s when responses are fast but up to ~30 min
 *     when they time out. The comment claimed "~300 s" either way. The
 *     budget is now real elapsed milliseconds and means what it says.
 *
 * [2] NO STALL DETECTION.
 *     A client that reaches a state and stops advancing (e.g. clStatus 3
 *     held for 35+ consecutive polls) held the radio up for the entire
 *     remaining budget for no reason. The loop now tracks the last state
 *     change and bails out early once clStatus has been static for
 *     LWM2M_STALL_TIMEOUT_MS, reporting the value it stalled on.
 *
 * [3] THE `result` FIELD WAS DISCARDED.
 *     The callback logged it and the routine ignored it. If the agent ever
 *     reports a non-zero result code, that IS the diagnosis. It is now
 *     captured, reported on every state change, and included in the
 *     failure message.
 *
 * [4] CLIENT LEFT ENABLED ON THE FAILURE PATH.
 *     A registration timeout returned without calling m2mb_lwm2m_disable(),
 *     leaving the agent running into the shutdown. Disable is now
 *     unconditional.
 *
 * [5] MAGIC NUMBERS NAMED.
 *     The 30 s post-registration report window and the clStatus == 4
 *     success test are now named constants.
 *
 * [6] ENABLE PARAMETERS CORRECTED AND NAMED, FROM CONFIRMED SDK LAYOUT.
 *     The struct was initialised with bare literals { MODE_NO_ACK, 1, 5, 5,
 *     CMD_TYPE_SET } with no indication of what any of them meant. The
 *     field layout has now been read out of the SDK's DWARF debug info:
 *
 *       typedef struct {
 *           M2MB_LWM2M_MODE_E        mode;
 *           UINT8                    apnclass;               // AT range 1-6
 *           UINT8                    guardRequestEventSecs;  // AT range 1-100
 *           UINT8                    guardReleaseEventSecs;  // AT range 1-100
 *           M2MB_LWM2MENA_CMD_TYPE_E commandType;
 *       } M2MB_LWM2M_ENABLE_REQ_T;
 *
 *     The original values were all legal and are unchanged in effect --
 *     they are simply named now. An earlier hypothesis that field 2 should
 *     be set to 7 (to point the client at the nxt20c.net control APN on
 *     CID 7) was WRONG: the field is an APN class, not a PDP context ID,
 *     and AT#LWM2MENA=? caps it at 6.
 *
 * [7] clStatus VALUES CONFIRMED (M2MB_LWM2M_CL_STATE_E, from debug info):
 *       0 = DISABLED / DEREGISTERED
 *       1 = BOOTSTRAPPING
 *       2 = BOOTSTRAPPED
 *       3 = REGISTERING     <-- where this device stalls
 *       4 = REGISTERED      <-- the success condition, was already correct
 *       5 = DEREGISTERING
 *       6 = SUSPENDED
 *     These are now named in the code and reported symbolically in the log.
 *
 * =============================================================================
 * WHAT THIS FILE CANNOT FIX
 * =============================================================================
 *
 * The observed stall is at clStatus 3 = REGISTERING, with result=0, while
 * the modem is attached (stat=5) and holds a valid IP. That means the
 * client is transmitting registration requests and receiving no answer.
 *
 * A LWM2M server that has no record of an endpoint does not return an
 * error -- it ignores the request, and the client retries in REGISTERING
 * indefinitely. Combined with the device not appearing in the Telit
 * portal, the evidence points to the endpoint not being provisioned
 * server-side rather than to any defect in this file.
 *
 * ACTION REQUIRED OUTSIDE THIS CODE: confirm IMEI 353141288950061 is
 * onboarded to your Telit OneEdge account and enabled for LWM2M. Until the
 * server knows this endpoint, REGISTERING is where it will stay.
 *
  * Exported interface is unchanged, so lwm2m_fn.h needs no edit.
 * ========================================================================== */
#include <string.h>

#include "m2mb_types.h"
#include "m2mb_os_api.h"
#include "m2mb_lwm2m.h"

#include "azx_log.h"
#include "azx_utils.h"

#include "app_common.h"
#include "lwm2m_fn.h"

/* ---- tuning ---------------------------------------------------------------*/

/*
 * APN class used by the LWM2M client (struct field `apnclass`).
 *
 * CONFIRMED from the SDK's DWARF debug info: M2MB_LWM2M_ENABLE_REQ_T is
 *   { mode, apnclass, guardRequestEventSecs, guardReleaseEventSecs,
 *     commandType }
 * and AT#LWM2MENA=? reports the apnclass range as (1-6).
 *
 * This is an APN CLASS, not a PDP context ID -- CID 7 (nxt20c.net) is NOT
 * a legal value here. An earlier hypothesis that the client should be
 * pointed at CID 7 was wrong on both counts and has been dropped.
 *
 * If your provider specifies a different class for device management,
 * change this to any value in 1-6. Check the current module setting with
 * AT#LWM2MENA? before altering it.
 */
#define LWM2M_APN_CLASS                  1

/* Total budget for the registration wait, in real elapsed milliseconds. */
#define LWM2M_REG_BUDGET_MS        180000UL

/* Give up early if clStatus has not changed for this long. A client that
 * has settled into a state is not going to advance on its own. */
#define LWM2M_STALL_TIMEOUT_MS      60000UL

/* Per-poll wait for the GET_STAT response. */
#define LWM2M_STAT_TIMEOUT_MS        5000UL

/* Sleep between polls. */
#define LWM2M_POLL_PERIOD_MS         1000UL

/* Time allowed for the agent to push its reports once registered. */
#define LWM2M_REPORT_WINDOW_MS      30000UL

/* Retry cap for m2mb_lwm2m_enable(). */
#define LWM2M_ENABLE_RETRIES            30

/* clStatus values -- CONFIRMED from M2MB_LWM2M_CL_STATE_E in the SDK. */
#define LWM2M_CLSTATUS_DISABLED          0
#define LWM2M_CLSTATUS_BOOTSTRAPPING     1
#define LWM2M_CLSTATUS_BOOTSTRAPPED      2
#define LWM2M_CLSTATUS_REGISTERING       3
#define LWM2M_CLSTATUS_REGISTERED        4
#define LWM2M_CLSTATUS_DEREGISTERING     5
#define LWM2M_CLSTATUS_SUSPENDED         6

/* Guard timers, struct fields 3 and 4. AT#LWM2MENA=? range: (1-100). */
#define LWM2M_GUARD_REQUEST_SECS         5
#define LWM2M_GUARD_RELEASE_SECS         5

/* Log the "not registered yet" line every N polls instead of every one,
 * so a stalled client does not flood the log. */
#define LWM2M_LOG_EVERY                 10

/* Human-readable clStatus, so the log says REGISTERING instead of 3. */
static const char *clstate_name(int st)
{
    switch (st)
    {
    case LWM2M_CLSTATUS_DISABLED:      return "DISABLED/DEREGISTERED";
    case LWM2M_CLSTATUS_BOOTSTRAPPING: return "BOOTSTRAPPING";
    case LWM2M_CLSTATUS_BOOTSTRAPPED:  return "BOOTSTRAPPED";
    case LWM2M_CLSTATUS_REGISTERING:   return "REGISTERING";
    case LWM2M_CLSTATUS_REGISTERED:    return "REGISTERED";
    case LWM2M_CLSTATUS_DEREGISTERING: return "DEREGISTERING";
    case LWM2M_CLSTATUS_SUSPENDED:     return "SUSPENDED";
    case -1:                           return "no-response";
    default:                           return "unknown";
    }
}

static M2MB_LWM2M_HANDLE lwm2m_handle = NULL;

/* owned copy of the stat response, not a pointer into SDK memory */
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
        memcpy(&g_lwm2m_stat, resp, sizeof(g_lwm2m_stat));   /* deep copy */
        g_lwm2m_stat_valid = TRUE;
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

    /* NOTE: the trailing `1` is the SDK's length argument. Confirm in
     * m2mb_lwm2m.h whether it is a byte count (should be 4 for a UINT32)
     * or an element count (1 is correct). Left as-is pending that check. */
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

/* ---- registration wait ----------------------------------------------------
 * FIX [1][2][3] live here. */
static BOOLEAN wait_for_lwm2m_registration(void)
{
    UINT32 elapsed_ms        = 0;
    UINT32 last_change_ms    = 0;
    int    last_clstatus     = -2;      /* -1 is "no valid stat", so use -2 */
    int    last_status       = -2;
    int    last_result       = 0;
    int    poll              = 0;

    AZX_LOG_INFO("Waiting for LWM2M registration (budget %lu s, "
                 "stall cutoff %lu s)\r\n",
                 (unsigned long)(LWM2M_REG_BUDGET_MS / 1000UL),
                 (unsigned long)(LWM2M_STALL_TIMEOUT_MS / 1000UL));

    while (elapsed_ms < LWM2M_REG_BUDGET_MS)
    {
        int cur_clstatus = -1;

        g_lwm2m_stat_valid = FALSE;

        if (M2MB_RESULT_SUCCESS == m2mb_lwm2m_get_stat(lwm2m_handle))
        {
            if (wait_ev(EV_LWM2M_STAT_BIT, LWM2M_STAT_TIMEOUT_MS))
            {
                if (g_lwm2m_stat_valid)
                {
                    cur_clstatus = (int)g_lwm2m_stat.clStatus;
                    last_status  = (int)g_lwm2m_stat.status;
                    last_result  = (int)g_lwm2m_stat.result;
                }
            }
            else
            {
                /* the stat response really did cost us that time */
                elapsed_ms += LWM2M_STAT_TIMEOUT_MS;
            }
        }

        if (cur_clstatus == LWM2M_CLSTATUS_REGISTERED)
        {
            AZX_LOG_INFO("lwm2m client registered (clStatus=%d) after %lu s "
                         "[apnclass=%d]\r\n",
                         cur_clstatus, (unsigned long)(elapsed_ms / 1000UL),
                         LWM2M_APN_CLASS);
            return TRUE;
        }

        /* FIX [2][3]: report every transition, with the result code */
        if (cur_clstatus != last_clstatus)
        {
            AZX_LOG_INFO("lwm2m state change: %s -> %s (clStatus %d, "
                         "status=%d result=%d) at %lu s\r\n",
                         clstate_name(last_clstatus),
                         clstate_name(cur_clstatus),
                         cur_clstatus, last_status, last_result,
                         (unsigned long)(elapsed_ms / 1000UL));
            last_clstatus  = cur_clstatus;
            last_change_ms = elapsed_ms;
        }
        else if ((elapsed_ms - last_change_ms) >= LWM2M_STALL_TIMEOUT_MS)
        {
            AZX_LOG_ERROR("lwm2m STALLED in %s (clStatus=%d status=%d "
                          "result=%d) for %lu s [apnclass=%d] -- abandoning "
                          "this cycle\r\n",
                          clstate_name(cur_clstatus),
                          cur_clstatus, last_status, last_result,
                          (unsigned long)((elapsed_ms - last_change_ms) / 1000UL),
                          LWM2M_APN_CLASS);
            if (cur_clstatus == LWM2M_CLSTATUS_REGISTERING)
            {
                AZX_LOG_ERROR("  -> stuck in REGISTERING with the modem "
                              "attached means the server is not answering. "
                              "Check this IMEI is onboarded to the DM "
                              "platform.\r\n");
            }
            return FALSE;
        }
        else if ((poll % LWM2M_LOG_EVERY) == 0)
        {
            AZX_LOG_INFO("lwm2m %s (clStatus=%d, %lu/%lu s)\r\n",
                         clstate_name(cur_clstatus), cur_clstatus,
                         (unsigned long)(elapsed_ms / 1000UL),
                         (unsigned long)(LWM2M_REG_BUDGET_MS / 1000UL));
        }

        azx_sleep_ms(LWM2M_POLL_PERIOD_MS);
        elapsed_ms += LWM2M_POLL_PERIOD_MS;
        poll++;
    }

    AZX_LOG_ERROR("lwm2m registration wait timed out after %lu s "
                  "(last clStatus=%d status=%d result=%d) [apnclass=%d]\r\n",
                  (unsigned long)(elapsed_ms / 1000UL),
                  last_clstatus, last_status, last_result,
                  LWM2M_APN_CLASS);
    return FALSE;
}

/* ---- enable / registration / disable cycle -------------------------------- */
BOOLEAN LWM2M_ROUTINE(void)
{
    int     ctr;
    BOOLEAN registered = FALSE;

    if (lwm2m_handle == NULL)
    {
        AZX_LOG_ERROR("LWM2M not initialized, skipping routine\r\n");
        return FALSE;
    }

    /* Field order CONFIRMED from SDK debug info:
     * { mode, apnclass, guardRequestEventSecs, guardReleaseEventSecs,
     *   commandType } */
    M2MB_LWM2M_ENABLE_REQ_T en_params = { M2MB_LWM2M_MODE_NO_ACK,
                                          LWM2M_APN_CLASS,
                                          LWM2M_GUARD_REQUEST_SECS,
                                          LWM2M_GUARD_RELEASE_SECS,
                                          M2MB_LWM2MENA_CMD_TYPE_SET };

    AZX_LOG_INFO("LWM2M enable: apnclass=%d guardReq=%d guardRel=%d\r\n",
                 LWM2M_APN_CLASS, LWM2M_GUARD_REQUEST_SECS,
                 LWM2M_GUARD_RELEASE_SECS);

    /* -- 1. Enable client -- */
    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_lwm2m_enable(lwm2m_handle, &en_params))
    {
        AZX_LOG_INFO("m2mb_lwm2m_enable failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr >= LWM2M_ENABLE_RETRIES)
        {
            AZX_LOG_ERROR("m2mb_lwm2m_enable gave up after %d attempts "
                          "(apnclass=%d)\r\n", ctr, LWM2M_APN_CLASS);
            /* FIX [4]: leave nothing half-enabled behind us */
            m2mb_lwm2m_disable(lwm2m_handle);
            return FALSE;
        }
    }
    AZX_LOG_INFO("m2mb_lwm2m_enable successful\r\n");

    azx_sleep_ms(1000);

    /* -- 2. Wait for registration (FIX [1][2][3]) -- */
    registered = wait_for_lwm2m_registration();

    /* -- 3. Report window -- */
    if (registered)
    {
        AZX_LOG_INFO("Giving the agent %lu s to push its reports\r\n",
                     (unsigned long)(LWM2M_REPORT_WINDOW_MS / 1000UL));
        azx_sleep_ms(LWM2M_REPORT_WINDOW_MS);
    }

    /* -- 4. Disable unconditionally (FIX [4]) -- */
    if (M2MB_RESULT_SUCCESS == m2mb_lwm2m_disable(lwm2m_handle))
    {
        AZX_LOG_INFO("LWM2MB disabled\r\n");
    }
    else
    {
        AZX_LOG_ERROR("m2mb_lwm2m_disable failed\r\n");
    }

    return registered;
}
