/* =============================================================================
 * net_fn.c  --  network + PDP bring-up
 *
 * =============================================================================
 * FIXES IN THIS REVISION (vs the previous net_fn.c)
 * =============================================================================
 *
 * [1] REGISTRATION BUDGET vs T3402  (the root cause of the perpetual
 *     "not connected to network, status:2" loop)
 *
 *     The old loop gave up after ~180 iterations (~3 min). The network
 *     broadcasts T3402 = 720 s (12 min) as the mandatory wait after a
 *     failed attach. The device therefore ALWAYS gave up inside the
 *     backoff window, powered off, and cold-started again -- it could
 *     never converge regardless of RF conditions.
 *
 *     Now: the budget is real elapsed milliseconds (not iterations), it
 *     is sized from the T3402 value actually read out of #RFSTS, and it
 *     defaults to 780 s so it clears the 720 s default.
 *
 * [2] MANUAL SELECTION NO LONGER FIRES DURING A BACKOFF
 *
 *     The old code ran AT+COPS=4 whenever ctr >= 60 && stat == 2. If the
 *     modem was camped on a cell and merely waiting out T3402, that
 *     manual selection interrupted the wait and restarted the cycle.
 *
 *     Now: #RFSTS is probed periodically. A populated #RFSTS line means a
 *     serving cell IS visible -> we are in a backoff, so we wait it out.
 *     Manual selection only runs when #RFSTS is empty (no cell at all),
 *     which is the case it was actually designed for.
 *
 * [3] EARLY EXIT ON REGISTRATION DENIED (stat == 3)
 *
 *     Status 3 is a subscription/roaming refusal. Waiting cannot fix it,
 *     so the routine now returns immediately instead of burning the full
 *     budget with the radio at full power.
 *
 * [4] NET_ROUTINE RETURNS THE TRUTH
 *
 *     The old code ended in an unconditional `return TRUE;`, so it could
 *     log "No IPv4 address obtained within timeout" and still report
 *     success. main.c's failure branch was effectively dead. The return
 *     value is now the real outcome (registered AND non-zero IP).
 *
 * [5] PDP ACTIVATION SKIPPED WHEN THE CONTEXT IS ALREADY UP
 *
 *     The network usually activates CID 1 during attach. The old code
 *     then hammered m2mb_pdp_activate() 120 times at 1 s each, failing
 *     every time with "context already activated" and burning two
 *     minutes of full-power awake time on a battery device. Status is
 *     now checked first, and the retry cap is 10 rather than 120.
 *
 * [6] RAT CONSTANTS WIDENED
 *
 *     The DNS workaround tested rat == 8 for Cat-M1. In 3GPP 27.007 the
 *     access-technology encoding is 7 = E-UTRAN, 8 = EC-GSM-IoT,
 *     9 = E-UTRAN NB-S1, so Cat-M1 commonly reports 7 and the old branch
 *     was dead code. Both 7 and 8 are now accepted. See the note at
 *     RAT_IS_CATM1 -- verify against m2mb_net.h for your SDK.
 *
 * -----------------------------------------------------------------------------
 * Preserved from the previous revision: deep-copied callback data, bounded
 * network list, g_reg_valid NULL guard, PDP after registration, COPS
 * persistence restore, explicit MNC width, fresh counters per loop,
 * non-zero IP requirement, single accumulating AT helper.
 *
 * NOTE: the exported interface is unchanged (NET_ROUTINE only), so this
 * file drops in without touching net_fn.h.
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "m2mb_types.h"
#include "m2mb_os_api.h"
#include "m2mb_net.h"
#include "m2mb_pdp.h"
#include "m2mb_socket.h"

#include "azx_log.h"
#include "azx_utils.h"

#include "app_common.h"
#include "at_helper.h"
#include "net_fn.h"
#include "sleep_fn.h"          /* REBOOT() */

#define PDP_CID          1
#define PDP_APN          "nxt20.net"
#define MAX_NW_ENTRIES   10

/* ---- registration timing -------------------------------------------------- */

/* Default EPS attach backoff broadcast by most networks, in seconds.
 * Used when T3402 cannot be read out of #RFSTS. */
#define T3402_DEFAULT_S             720UL

/* Extra headroom added on top of T3402 so we are still awake when the
 * backoff expires and the modem retries. */
#define NET_REG_BACKOFF_MARGIN_MS   120000UL

/* Hard ceiling on how long the registration wait may keep the radio up.
 * Raising this costs battery; lowering it below T3402 reintroduces the
 * original bug. */
#define NET_REG_BUDGET_MAX_MS       900000UL

/* Used until T3402 has been read (clears the 720 s default). */
#define NET_REG_BUDGET_DEFAULT_MS   780000UL

#define NET_REG_POLL_TIMEOUT_MS     5000UL   /* wait for one reg-status resp  */
#define NET_REG_POLL_PERIOD_MS      1000UL   /* sleep between polls           */

/* How often to spend an AT round-trip probing #RFSTS for a serving cell. */
#define NET_RFSTS_PROBE_EVERY       15       /* iterations (~15 s)            */

/* Only consider manual PLMN selection after this long with NO serving cell. */
#define NET_MANUAL_TRIGGER_MS       90000UL

/* Log the "still waiting" line every N iterations instead of every one. */
#define NET_LOG_EVERY               10

/* Zero-based comma index of T3402 in the LTE #RFSTS response:
 * PLMN,EARFCN,RSRP,RSSI,RSRQ,TAC,RAC,TXPWR,DRX,MM,RRC,CID,IMSI,
 * NetNameAsc,SD,ABND,T3402,T3412,SINR                                      */
#define RFSTS_T3402_FIELD           16

/* ---- RAT classification ---------------------------------------------------
 * 3GPP 27.007 <AcT>: 7 = E-UTRAN, 8 = EC-GSM-IoT, 9 = E-UTRAN NB-S1.
 * Cat-M1 is normally reported as 7. The previous revision only tested 8.
 * Both are accepted here; confirm the actual m2mb enum in m2mb_net.h and
 * narrow these if your SDK differs. */
#define RAT_IS_CATM1(r)  (((r) == 7) || ((r) == 8))
#define RAT_IS_NBIOT(r)  ((r) == 9)

/* ---- module state -------------------------------------------------------- */
static M2MB_NET_HANDLE network_handle = NULL;
static M2MB_PDP_HANDLE pdp_handle     = NULL;

/* structs (owned copies), not pointers into SDK callback memory */
static M2MB_NET_REG_STATUS_T             g_reg;
static BOOLEAN                           g_reg_valid = FALSE;
static M2MB_NET_GET_SIGNAL_INFO_RESP_T   g_signal;
static M2MB_NET_GET_CURRENT_OPERATOR_INFO_RESP_T g_op;
static M2MB_NET_MODE_PREFERENCE_STATUS_T g_mode_pref;

/* bounded, deep-copied network scan results (no linked-list pointers) */
typedef struct
{
    UINT16 mcc;
    UINT16 mnc;
    UINT16 rat;
    M2MB_NET_AVAILABILITY_E av;
} NW_ENTRY_T;

static NW_ENTRY_T          g_nw[MAX_NW_ENTRIES];
static UINT16              g_nw_count = 0;
static M2MB_NET_ERR_CAUSE_E g_nw_err  = M2MB_NET_ERR_OK;

/* Last T3402 seen, in seconds. 0 = not read yet. */
static UINT32 g_t3402_s = 0;

/* ---- network callback ---------------------------------------------------- */
static void Network_callback(M2MB_NET_HANDLE h, M2MB_NET_IND_E net_event,
                             UINT16 resp_size, void *resp_struct, void *myUserdata)
{
    (void)h; (void)resp_size; (void)myUserdata;

    switch (net_event)
    {
    case M2MB_NET_GET_REG_STATUS_INFO_RESP:
    {
        M2MB_NET_REG_STATUS_T *p = (M2MB_NET_REG_STATUS_T *)resp_struct;
        memcpy(&g_reg, p, sizeof(g_reg));           /* deep copy */
        g_reg_valid = TRUE;
        AZX_LOG_INFO("net reg: status:%d, rat:%d, areacode:%d, cellid:%d\r\n",
                     g_reg.stat, g_reg.rat, g_reg.areaCode, g_reg.cellID);
        set_ev(EV_NET_REG_BIT);
        break;
    }
    case M2MB_NET_GET_SIGNAL_INFO_RESP:
    {
        M2MB_NET_GET_SIGNAL_INFO_RESP_T *p =
            (M2MB_NET_GET_SIGNAL_INFO_RESP_T *)resp_struct;
        memcpy(&g_signal, p, sizeof(g_signal));
        AZX_LOG_INFO("sig info: rssi:%d\r\n", g_signal.rssi);
        set_ev(EV_NET_SIG_BIT);
        break;
    }
    case M2MB_NET_GET_BER_RESP:
    {
        set_ev(EV_NET_BER_BIT);
        break;
    }
    case M2MB_NET_GET_CURRENT_OPERATOR_INFO_RESP:
    {
        M2MB_NET_GET_CURRENT_OPERATOR_INFO_RESP_T *p =
            (M2MB_NET_GET_CURRENT_OPERATOR_INFO_RESP_T *)resp_struct;
        memcpy(&g_op, p, sizeof(g_op));
        AZX_LOG_INFO("op info: mcc:%d, mnc:%d\r\n", g_op.mcc, g_op.mnc);
        set_ev(EV_NET_OP_BIT);
        break;
    }
    case M2MB_NET_GET_MODE_PREFERENCE_RESP:
    {
        M2MB_NET_MODE_PREFERENCE_STATUS_T *p =
            (M2MB_NET_MODE_PREFERENCE_STATUS_T *)resp_struct;
        memcpy(&g_mode_pref, p, sizeof(g_mode_pref));
        AZX_LOG_INFO("mode pref: selectedwirelessnetwork:%d, lteciotpref:%d\r\n",
                     g_mode_pref.mode_preference.selectedWirelessNetwork,
                     g_mode_pref.mode_preference.lte_ciot_preference);
        set_ev(EV_NET_MODE_BIT);
        break;
    }
    case M2MB_NET_GET_AVAILABLE_NW_LIST_RESP:
    {
        M2MB_NET_GET_AVAILABLE_NW_LIST_RESP_T *p =
            (M2MB_NET_GET_AVAILABLE_NW_LIST_RESP_T *)resp_struct;

        /* deep copy each node's fields NOW, while the SDK-owned linked
         * list is still valid. Never keep the pointers. */
        g_nw_err   = p->err;
        g_nw_count = 0;

        M2MB_NET_DESCRIPTION_T *nw = p->availableNetworks;
        while (nw != NULL && g_nw_count < MAX_NW_ENTRIES)
        {
            const char *avail_str = "unknown";
            switch (nw->networkAv)
            {
                case M2MB_NET_AVAILABILITY_AVAILABLE: avail_str = "available"; break;
                case M2MB_NET_AVAILABILITY_CURRENT:   avail_str = "current";   break;
                case M2MB_NET_AVAILABILITY_FORBIDDEN: avail_str = "forbidden"; break;
                default: break;
            }
            AZX_LOG_INFO("  NW: mcc=%d mnc=%d rat=%d status=%s\r\n",
                         nw->mcc, nw->mnc, nw->rat, avail_str);

            g_nw[g_nw_count].mcc = nw->mcc;
            g_nw[g_nw_count].mnc = nw->mnc;
            g_nw[g_nw_count].rat = nw->rat;
            g_nw[g_nw_count].av  = nw->networkAv;
            g_nw_count++;

            nw = nw->next;
        }
        AZX_LOG_INFO("available networks: count=%d (stored %d)\r\n",
                     p->availableNetworks_size, g_nw_count);
        set_ev(EV_NW_LIST_BIT);
        break;
    }
    default:
        break;
    }
}

static void m2mb_pdp_callback(M2MB_PDP_HANDLE h, M2MB_PDP_IND_E pdp_event,
                              UINT8 cid, void *myUserdata)
{
    (void)h; (void)pdp_event; (void)cid; (void)myUserdata;
}

/* ---- helpers ------------------------------------------------------------- */

/* Request a fresh registration status and wait for the response.
 * On success g_reg / g_reg_valid are updated. */
static BOOLEAN refresh_reg_status(UINT32 timeout_ms)
{
    if (M2MB_RESULT_SUCCESS != m2mb_net_get_reg_status_info(network_handle))
        return FALSE;
    return wait_ev(EV_NET_REG_BIT, timeout_ms);
}

/*
 * FIX [2] helper: does the modem currently see a serving cell?
 *
 * #RFSTS answers with a bare OK when there is no serving cell, and with a
 * populated "#RFSTS: ..." line once it is camped. That distinction is the
 * difference between "no coverage / wrong band" (manual scan is useful)
 * and "camped but waiting out an attach backoff" (manual scan is harmful).
 */
static BOOLEAN rfsts_probe(char *resp, UINT16 resp_size)
{
    if (resp == NULL || resp_size == 0)
        return FALSE;

    resp[0] = '\0';
    if (!ati_send_and_wait("AT#RFSTS\r", resp, resp_size, 5000))
        return FALSE;

    return (strstr(resp, "#RFSTS:") != NULL) ? TRUE : FALSE;
}

/*
 * FIX [1] helper: pull T3402 (attach backoff, seconds) out of an #RFSTS
 * response. Returns 0 if the field is absent or implausible, in which case
 * the caller falls back to T3402_DEFAULT_S.
 *
 * Quoted #RFSTS fields ("262 03", "o2 - de", the IMSI) contain no commas,
 * so plain comma counting is safe here.
 */
static UINT32 rfsts_get_t3402(const char *resp)
{
    const char *p;
    int commas = 0;

    if (resp == NULL)
        return 0;

    p = strstr(resp, "#RFSTS:");
    if (p == NULL)
        return 0;
    p += 7;                                  /* skip "#RFSTS:" */

    while (*p != '\0' && *p != '\r' && *p != '\n')
    {
        if (*p == ',')
        {
            commas++;
            if (commas == RFSTS_T3402_FIELD)
            {
                unsigned long v = strtoul(p + 1, NULL, 10);
                if (v >= 30UL && v <= 7200UL)
                    return (UINT32)v;
                return 0;
            }
        }
        p++;
    }
    return 0;
}

/*
 * COPS persistence: AT+COPS=4 from a previous cycle survives reboot on
 * Telit modules, permanently pinning the modem to one PLMN. Restore
 * automatic selection at the start of every cycle so "auto first, manual
 * fallback" actually holds on every wake, not just the first one.
 */
static void restore_auto_plmn_selection(void)
{
    char resp[256];

    if (!ati_send_and_wait("AT+COPS?\r", resp, sizeof(resp), 10000))
    {
        AZX_LOG_INFO("AT+COPS? failed, skipping auto-restore check\r\n");
        return;
    }

    char *p = strstr(resp, "+COPS:");
    if (p != NULL)
    {
        int mode = atoi(p + 6);   /* atoi skips the leading space */
        if (mode != 0)
        {
            AZX_LOG_INFO("COPS mode=%d left over from previous cycle, "
                         "restoring automatic selection\r\n", mode);
            /* COPS=0 can take a while while the modem re-registers */
            ati_send_and_wait("AT+COPS=0\r", resp, sizeof(resp), 60000);
        }
    }
}

/* ---- manual network selection fallback -----------------------------------
 * Only called when NO serving cell is visible (see FIX [2]). */
static BOOLEAN AUTOMATED_MANUAL_NWSELECTION_ROUTINE(void)
{
    UINT16 i;

    AZX_LOG_INFO("No serving cell visible, scanning available networks...\r\n");

    if (M2MB_RESULT_SUCCESS != m2mb_net_get_available_nw_list(network_handle))
    {
        AZX_LOG_ERROR("m2mb_net_get_available_nw_list request failed\r\n");
        return FALSE;
    }

    /* Scan can take 30-60 s; wait up to 90 s */
    if (!wait_ev(EV_NW_LIST_BIT, 90000))
    {
        AZX_LOG_ERROR("Network scan timed out\r\n");
        return FALSE;
    }

    if (g_nw_err != M2MB_NET_ERR_OK || g_nw_count == 0)
    {
        AZX_LOG_ERROR("No available networks found (err=%d count=%d)\r\n",
                      g_nw_err, g_nw_count);
        return FALSE;
    }

    for (i = 0; i < g_nw_count; i++)
    {
        char cmd[64];
        char resp[256];
        int poll;
        BOOLEAN registered = FALSE;

        if (g_nw[i].av == M2MB_NET_AVAILABILITY_FORBIDDEN)
            continue;

        AZX_LOG_INFO("Trying: mcc=%d mnc=%d rat=%d\r\n",
                     g_nw[i].mcc, g_nw[i].mnc, g_nw[i].rat);

        /* explicit 2-digit vs 3-digit MNC formatting */
        if (g_nw[i].mnc < 100)
            snprintf(cmd, sizeof(cmd), "AT+COPS=4,2,\"%03d%02d\",%d\r",
                     g_nw[i].mcc, g_nw[i].mnc, g_nw[i].rat);
        else
            snprintf(cmd, sizeof(cmd), "AT+COPS=4,2,\"%03d%03d\",%d\r",
                     g_nw[i].mcc, g_nw[i].mnc, g_nw[i].rat);

        AZX_LOG_INFO("Sending: %s\r\n", cmd);

        /* COPS with manual selection can take up to 90 s to answer */
        if (!ati_send_and_wait(cmd, resp, sizeof(resp), 90000))
        {
            AZX_LOG_INFO("COPS rejected for mcc=%d mnc=%d, trying next...\r\n",
                         g_nw[i].mcc, g_nw[i].mnc);
            continue;
        }

        /* COPS OK only means the modem accepted the command; now poll
         * registration status until stat 1/5 or ~40 s elapse. */
        AZX_LOG_INFO("COPS OK for %d/%d, waiting for actual registration...\r\n",
                     g_nw[i].mcc, g_nw[i].mnc);
        azx_sleep_ms(5000);

        for (poll = 0; poll < 20; poll++)
        {
            if (refresh_reg_status(5000))
            {
                if (g_reg_valid && (g_reg.stat == 1 || g_reg.stat == 5))
                {
                    registered = TRUE;
                    break;
                }
                /* FIX [3]: denied here too -- no point trying this PLMN
                 * any longer, move on to the next one immediately. */
                if (g_reg_valid && g_reg.stat == 3)
                {
                    AZX_LOG_INFO("Registration DENIED on mcc=%d mnc=%d\r\n",
                                 g_nw[i].mcc, g_nw[i].mnc);
                    break;
                }
            }
            azx_sleep_ms(2000);
        }

        if (registered)
        {
            AZX_LOG_INFO("=== Registered on mcc=%d mnc=%d (stat=%d) ===\r\n",
                         g_nw[i].mcc, g_nw[i].mnc, g_reg.stat);
            return TRUE;
        }

        AZX_LOG_INFO("Registration failed on mcc=%d mnc=%d, trying next...\r\n",
                     g_nw[i].mcc, g_nw[i].mnc);
    }

    AZX_LOG_ERROR("All available networks exhausted, none registered\r\n");
    return FALSE;
}

/* ---- NB-IoT DNS / CCIOTOPT workaround ------------------------------------ */
/*
 * - If on NB-IoT and DNS is missing: set AT#CCIOTOPT=0437, reboot
 * - If on Cat-M1 and CCIOTOPT is 0437: restore to 0537, reboot
 * - Otherwise: do nothing
 */
static void check_nbiot_dns_workaround(void)
{
    char resp[256];
    char cmd[64];
    int current_rat;
    BOOLEAN dns_present = FALSE;
    int current_opt = -1;
    BOOLEAN need_reboot = FALSE;

    if (!g_reg_valid)
    {
        AZX_LOG_INFO("No valid reg status, skipping DNS workaround check\r\n");
        return;
    }
    current_rat = g_reg.rat;
    AZX_LOG_INFO("Current RAT: %d (Cat-M1=7/8, NB-IoT=9)\r\n", current_rat);

    /* Step 1: Check DNS via AT+CGCONTRDP=1 (accumulated response, so the
     * payload line survives even if "OK" arrives in a later chunk) */
    snprintf(cmd, sizeof(cmd), "AT+CGCONTRDP=%d\r", PDP_CID);
    if (ati_send_and_wait(cmd, resp, sizeof(resp), 5000))
    {
        /* Response: +CGCONTRDP: 1,5,"apn","ip",,<dns1>,<dns2>
         * Count commas; if the char after the 5th comma is a value,
         * DNS exists. */
        char *p = strstr(resp, "+CGCONTRDP:");
        if (p != NULL)
        {
            int commas = 0;
            while (*p && commas < 5)
            {
                if (*p == ',') commas++;
                p++;
            }
            if (commas >= 5 && *p != ',' && *p != '\r' && *p != '\n' && *p != '\0')
            {
                dns_present = TRUE;
            }
        }
    }
    AZX_LOG_INFO("DNS present: %s\r\n", dns_present ? "YES" : "NO");

    /* Step 2: Read current CCIOTOPT */
    if (ati_send_and_wait("AT#CCIOTOPT?\r", resp, sizeof(resp), 5000))
    {
        char *p = strstr(resp, "#CCIOTOPT:");
        if (p != NULL)
        {
            p += 10;                     /* skip "#CCIOTOPT:" */
            while (*p == ' ') p++;       /* tolerate 0/1 spaces */
            current_opt = (int)strtol(p, NULL, 16);
        }
    }
    AZX_LOG_INFO("Current CCIOTOPT: 0x%04X\r\n", current_opt);

    /* Step 3: Decide (FIX [6]: RAT test widened) */
    if (RAT_IS_NBIOT(current_rat) && !dns_present && current_opt != 0x0437)
    {
        AZX_LOG_INFO("NB-IoT without DNS detected, setting CCIOTOPT=0437\r\n");
        if (ati_send_and_wait("AT#CCIOTOPT=0437\r", resp, sizeof(resp), 5000))
        {
            AZX_LOG_INFO("CCIOTOPT set to 0437 OK\r\n");
            need_reboot = TRUE;
        }
    }
    else if (RAT_IS_CATM1(current_rat) && current_opt == 0x0437)
    {
        AZX_LOG_INFO("Cat-M1 with CCIOTOPT=0437, restoring to 0537\r\n");
        if (ati_send_and_wait("AT#CCIOTOPT=0537\r", resp, sizeof(resp), 5000))
        {
            AZX_LOG_INFO("CCIOTOPT restored to 0537 OK\r\n");
            need_reboot = TRUE;
        }
    }
    else
    {
        AZX_LOG_INFO("CCIOTOPT correct for current RAT/DNS state, no change\r\n");
    }

    if (need_reboot)
    {
        AZX_LOG_INFO("Rebooting to apply CCIOTOPT change...\r\n");
        azx_sleep_ms(3000);
        REBOOT();
    }
}

/* ---- registration wait ----------------------------------------------------
 * FIX [1][2][3] live here. Returns TRUE once stat is 1 or 5. */
static BOOLEAN wait_for_registration(void)
{
    char      rfsts[320];
    UINT32    elapsed_ms   = 0;
    UINT32    budget_ms    = NET_REG_BUDGET_DEFAULT_MS;
    BOOLEAN   cell_seen    = FALSE;
    BOOLEAN   manual_tried = FALSE;
    BOOLEAN   budget_sized = FALSE;
    int       iter         = 0;

    AZX_LOG_INFO("Waiting for registration (initial budget %lu s)\r\n",
                 (unsigned long)(budget_ms / 1000UL));

    while (elapsed_ms < budget_ms)
    {
        if (!refresh_reg_status(NET_REG_POLL_TIMEOUT_MS))
        {
            /* the request or the response timed out; that time really passed */
            elapsed_ms += NET_REG_POLL_TIMEOUT_MS;
        }

        if (g_reg_valid && (g_reg.stat == 1 || g_reg.stat == 5))
        {
            AZX_LOG_INFO("Registered (stat=%d) after %lu s\r\n",
                         g_reg.stat, (unsigned long)(elapsed_ms / 1000UL));
            return TRUE;
        }

        /* FIX [3]: denied is terminal, do not burn the budget on it */
        if (g_reg_valid && g_reg.stat == 3)
        {
            AZX_LOG_ERROR("Registration DENIED (stat=3). SIM/roaming "
                          "provisioning problem -- aborting this cycle.\r\n");
            return FALSE;
        }

        if ((iter % NET_LOG_EVERY) == 0)
        {
            if (g_reg_valid)
                AZX_LOG_INFO("not connected to network, status:%d (%lu/%lu s)\r\n",
                             g_reg.stat,
                             (unsigned long)(elapsed_ms / 1000UL),
                             (unsigned long)(budget_ms / 1000UL));
            else
                AZX_LOG_INFO("no registration status yet (%lu s)\r\n",
                             (unsigned long)(elapsed_ms / 1000UL));
        }

        /* ---- periodic #RFSTS probe: cell visible or not? ---------------- */
        if ((iter % NET_RFSTS_PROBE_EVERY) == 0)
        {
            BOOLEAN now_seen = rfsts_probe(rfsts, sizeof(rfsts));

            if (now_seen && !cell_seen)
            {
                AZX_LOG_INFO("Serving cell now visible -- modem is camped, "
                             "attach pending\r\n");
            }
            cell_seen = now_seen;

            /* FIX [1]: size the budget from the network's own T3402 */
            if (cell_seen && !budget_sized)
            {
                UINT32 t = rfsts_get_t3402(rfsts);
                if (t == 0)
                {
                    t = T3402_DEFAULT_S;
                    AZX_LOG_INFO("T3402 not parsed, assuming %lu s\r\n",
                                 (unsigned long)t);
                }
                else
                {
                    AZX_LOG_INFO("T3402 from #RFSTS: %lu s\r\n",
                                 (unsigned long)t);
                }
                g_t3402_s = t;

                budget_ms = ((UINT32)t * 1000UL) + NET_REG_BACKOFF_MARGIN_MS;
                if (budget_ms > NET_REG_BUDGET_MAX_MS)
                    budget_ms = NET_REG_BUDGET_MAX_MS;
                budget_sized = TRUE;

                AZX_LOG_INFO("Registration budget set to %lu s\r\n",
                             (unsigned long)(budget_ms / 1000UL));
            }
        }

        /* ---- FIX [2]: manual selection ONLY when no cell is visible ----- */
        if (!manual_tried
            && !cell_seen
            && elapsed_ms >= NET_MANUAL_TRIGGER_MS
            && g_reg_valid
            && g_reg.stat == 2)
        {
            manual_tried = TRUE;
            AZX_LOG_INFO("No cell after %lu s, trying manual selection...\r\n",
                         (unsigned long)(elapsed_ms / 1000UL));
            if (AUTOMATED_MANUAL_NWSELECTION_ROUTINE())
            {
                AZX_LOG_INFO("letting modem settle...\r\n");
                azx_sleep_ms(5000);
                elapsed_ms += 5000;
            }
        }
        else if (!manual_tried && cell_seen && elapsed_ms >= NET_MANUAL_TRIGGER_MS
                 && (iter % NET_LOG_EVERY) == 0)
        {
            /* This is the case the old code got wrong: a cell IS there, so
             * COPS=4 would only interrupt the backoff. Wait it out. */
            AZX_LOG_INFO("Cell visible but not attached -- waiting out attach "
                         "backoff, NOT forcing manual selection\r\n");
        }

        azx_sleep_ms(NET_REG_POLL_PERIOD_MS);
        elapsed_ms += NET_REG_POLL_PERIOD_MS;
        iter++;
    }

    AZX_LOG_ERROR("Network registration failed after %lu s "
                  "(cell_seen=%d, manual_tried=%d)\r\n",
                  (unsigned long)(elapsed_ms / 1000UL),
                  (int)cell_seen, (int)manual_tried);
    return FALSE;
}

/* ---- PDP bring-up ---------------------------------------------------------
 * FIX [5]: do not fight an already-active context. */
static BOOLEAN bring_up_pdp(void)
{
    UINT8 pdp_stat = 0;
    int   ctr;

    if (M2MB_RESULT_SUCCESS == m2mb_pdp_get_status(pdp_handle, PDP_CID, &pdp_stat)
        && pdp_stat != 0)
    {
        AZX_LOG_INFO("PDP CID %d already active (stat=%d), skipping activation\r\n",
                     PDP_CID, pdp_stat);
        return TRUE;
    }

    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_pdp_activate(pdp_handle, PDP_CID,
                                                    (CHAR *)PDP_APN,
                                                    (CHAR *)"", (CHAR *)"",
                                                    M2MB_PDP_IPV4))
    {
        /* The network may have activated it underneath us between the
         * status read and this call -- re-check before counting a failure. */
        pdp_stat = 0;
        if (M2MB_RESULT_SUCCESS == m2mb_pdp_get_status(pdp_handle, PDP_CID,
                                                       &pdp_stat)
            && pdp_stat != 0)
        {
            AZX_LOG_INFO("PDP CID %d came up during activation\r\n", PDP_CID);
            return TRUE;
        }

        AZX_LOG_INFO("m2mb_pdp_activate failed (attempt %d)\r\n", ctr + 1);
        azx_sleep_ms(1000);

        /* was 120 -- two wasted minutes per cycle on a battery device */
        if (++ctr >= 10)
        {
            AZX_LOG_ERROR("m2mb_pdp_activate gave up after %d attempts\r\n", ctr);
            return FALSE;
        }
    }

    AZX_LOG_INFO("m2mb_pdp_activate succeeded\r\n");

    pdp_stat = 0;
    if (M2MB_RESULT_SUCCESS == m2mb_pdp_get_status(pdp_handle, PDP_CID, &pdp_stat))
    {
        AZX_LOG_INFO("pdp stat:%d\r\n", pdp_stat);
    }
    return TRUE;
}

/* ---- main network routine ------------------------------------------------- */
BOOLEAN NET_ROUTINE(void)
{
    int     ctr;
    BOOLEAN got_ip = FALSE;

    g_reg_valid = FALSE;
    g_t3402_s   = 0;

    /* -- 1. Init NET (bounded) -- */
    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_net_init(&network_handle,
                                                Network_callback, NULL))
    {
        AZX_LOG_INFO("m2mb_net_init failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 30)
        {
            AZX_LOG_ERROR("m2mb_net_init gave up\r\n");
            return FALSE;
        }
    }
    AZX_LOG_INFO("m2mb_net_init success\r\n");

    /* -- 2. Init PDP -- */
    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_pdp_init(&pdp_handle,
                                                m2mb_pdp_callback, NULL))
    {
        AZX_LOG_INFO("m2mb_pdp_init failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 30)
        {
            AZX_LOG_ERROR("m2mb_pdp_init gave up\r\n");
            return FALSE;
        }
    }
    AZX_LOG_INFO("m2mb_pdp_init success\r\n");

    /* -- 3. If a previous cycle left the modem in manual PLMN mode,
     *       restore automatic selection first -- */
    restore_auto_plmn_selection();

    /* -- 4. Informational: mode preference + signal (bounded waits) -- */
    if (M2MB_RESULT_SUCCESS == m2mb_net_get_mode_preference(network_handle))
    {
        wait_ev(EV_NET_MODE_BIT, 5000);
    }
    if (M2MB_RESULT_SUCCESS == m2mb_net_get_signal_info(network_handle))
    {
        wait_ev(EV_NET_SIG_BIT, 5000);
    }

    /* -- 5. Wait for registration (FIX [1][2][3]) -- */
    if (!wait_for_registration())
    {
        /* FIX [4]: report the real outcome */
        return FALSE;
    }
    AZX_LOG_INFO("Network connection successful!!!\r\n");

    /* -- 6. Activate PDP only if it is not already up (FIX [5]) -- */
    if (!bring_up_pdp())
    {
        AZX_LOG_ERROR("PDP bring-up failed\r\n");
        /* keep going: the IP check below is the authoritative test */
    }

    /* -- 7. Wait for a real (non-zero) IP address -- */
    {
        M2MB_SOCKET_BSD_IN_ADDR addrv4;
        addrv4.s_addr = 0;
        ctr = 0;                       /* fresh counter */
        while (ctr <= 120)
        {
            if (M2MB_RESULT_SUCCESS == m2mb_pdp_get_my_ip(pdp_handle, PDP_CID,
                                                          M2MB_PDP_IPV4, &addrv4)
                && addrv4.s_addr != 0) /* 0.0.0.0 keeps waiting */
            {
                UINT8 *ip = (UINT8 *)&addrv4.s_addr;
                AZX_LOG_INFO("=== IP address: %d.%d.%d.%d ===\r\n",
                             ip[0], ip[1], ip[2], ip[3]);
                got_ip = TRUE;
                break;
            }
            azx_sleep_ms(1000);
            ctr++;
        }
        if (!got_ip)
            AZX_LOG_ERROR("No IPv4 address obtained within timeout\r\n");
    }

    /* -- 8. NB-IoT DNS workaround (may REBOOT and not return) -- */
    check_nbiot_dns_workaround();

    /* FIX [4]: TRUE only when the network is genuinely usable */
    return got_ip;
}

/* =============================================================================
 * NET_WAIT_REGISTERED
 *
 * Exported so main.c can confirm the modem is actually back on the network
 * after GNSS has released the radio, instead of sleeping a fixed interval
 * and hoping.
 *
 * GNSS_ROUTINE switches the module to GNSS priority, which on this
 * shared-RF part takes the cellular link down for as long as the fix
 * attempt runs -- up to several minutes on a timeout. A blind settle delay
 * that is adequate after a 40 s fix is far too short after a 5 min one,
 * which is the suspected cause of the LWM2M client stalling at clStatus 3.
 *
 * Returns TRUE as soon as registration status reads 1 (home) or 5
 * (roaming). Returns FALSE on denial (stat 3, terminal) or timeout.
 *
 * Safe to call before NET_ROUTINE has run: returns FALSE if the network
 * handle was never initialised.
 * ========================================================================== */
BOOLEAN NET_WAIT_REGISTERED(UINT32 timeout_ms)
{
    UINT32 elapsed_ms = 0;
    int    iter       = 0;

    if (network_handle == NULL)
    {
        AZX_LOG_ERROR("NET_WAIT_REGISTERED: network not initialised\r\n");
        return FALSE;
    }

    AZX_LOG_INFO("Waiting for the modem to re-attach (up to %lu s)...\r\n",
                 (unsigned long)(timeout_ms / 1000UL));

    while (elapsed_ms < timeout_ms)
    {
        if (!refresh_reg_status(NET_REG_POLL_TIMEOUT_MS))
        {
            elapsed_ms += NET_REG_POLL_TIMEOUT_MS;
        }

        if (g_reg_valid && (g_reg.stat == 1 || g_reg.stat == 5))
        {
            AZX_LOG_INFO("Modem re-attached (stat=%d) after %lu s\r\n",
                         g_reg.stat, (unsigned long)(elapsed_ms / 1000UL));
            return TRUE;
        }

        if (g_reg_valid && g_reg.stat == 3)
        {
            AZX_LOG_ERROR("NET_WAIT_REGISTERED: registration DENIED (stat=3)\r\n");
            return FALSE;
        }

        if ((iter % 10) == 0)
        {
            AZX_LOG_INFO("  still re-attaching (stat=%d, %lu/%lu s)\r\n",
                         g_reg_valid ? g_reg.stat : -1,
                         (unsigned long)(elapsed_ms / 1000UL),
                         (unsigned long)(timeout_ms / 1000UL));
        }

        azx_sleep_ms(NET_REG_POLL_PERIOD_MS);
        elapsed_ms += NET_REG_POLL_PERIOD_MS;
        iter++;
    }

    AZX_LOG_ERROR("NET_WAIT_REGISTERED: modem did not re-attach within %lu s "
                  "(last stat=%d)\r\n",
                  (unsigned long)(timeout_ms / 1000UL),
                  g_reg_valid ? g_reg.stat : -1);
    return FALSE;
}
