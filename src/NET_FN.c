/* =============================================================================
 * net_fn.c
 *
 * FIXES vs original main.c:
 *  - Callback data is DEEP-COPIED into module-owned structs inside the
 *    callback (old code stored raw resp_struct POINTERS globally and
 *    dereferenced them later from the main thread = use-after-scope).
 *  - Available-network list is deep-copied into a bounded array (old code
 *    shallow-copied the response struct and later walked a linked list
 *    whose nodes lived in callback-owned memory = use-after-free).
 *  - NULL-deref guard: registration status is only trusted after the first
 *    successful response (g_reg_valid), so a failed first request can no
 *    longer crash the stat-polling loop.
 *  - PDP is activated AFTER registration is confirmed (old code burned up
 *    to 120 doomed activation retries before the modem was registered).
 *  - Manual-selection fallback triggers on ctr >= 60 with a one-shot flag
 *    (old code required ctr == 60 EXACTLY while stat happened to be 2).
 *  - AT+COPS persistence fix: at the start of every cycle, if the modem is
 *    still pinned to a manual PLMN from a previous cycle, automatic
 *    selection (AT+COPS=0) is restored first. Old code left the modem in
 *    manual mode forever after the first fallback.
 *  - MNC formatting is explicit: %02d for 2-digit MNC, %03d for 3-digit
 *    (old code had two identical branches / accidental correctness).
 *  - ctr is reset before every loop that uses it (old code leaked the
 *    registration counter into the IP-wait loop, silently shortening it).
 *  - IP wait loop only exits early when a NON-ZERO address is returned
 *    (old code broke on any successful API call, even for 0.0.0.0).
 *  - All AT interactions go through ati_send_and_wait() (accumulating
 *    buffer) instead of three duplicated overwrite-prone inline loops.
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
#include "SLEEP_FN.h"          /* REBOOT() */

#define PDP_CID          1
#define PDP_APN          "nxt20.net"
#define MAX_NW_ENTRIES   10

/* ---- module state -------------------------------------------------------- */
static M2MB_NET_HANDLE network_handle = NULL;
static M2MB_PDP_HANDLE pdp_handle     = NULL;

/* FIX: structs (owned copies), not pointers into SDK callback memory */
static M2MB_NET_REG_STATUS_T             g_reg;
static BOOLEAN                           g_reg_valid = FALSE;
static M2MB_NET_GET_SIGNAL_INFO_RESP_T   g_signal;
static M2MB_NET_GET_CURRENT_OPERATOR_INFO_RESP_T g_op;
static M2MB_NET_MODE_PREFERENCE_STATUS_T g_mode_pref;

/* FIX: bounded, deep-copied network scan results (no linked-list pointers) */
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
        memcpy(&g_reg, p, sizeof(g_reg));           /* FIX: deep copy */
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

        /* FIX: deep copy each node's fields NOW, while the SDK-owned
         * linked list is still valid. Never keep the pointers. */
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
 * FIX (COPS persistence): AT+COPS=4 from a previous cycle survives reboot
 * on Telit modules, permanently pinning the modem to one PLMN. Restore
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

/* ---- manual network selection fallback ----------------------------------- */
static BOOLEAN AUTOMATED_MANUAL_NWSELECTION_ROUTINE(void)
{
    UINT16 i;

    AZX_LOG_INFO("Auto-registration failed, scanning available networks...\r\n");

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

        /* FIX: explicit 2-digit vs 3-digit MNC formatting */
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
 * - If on NB-IoT (rat 9) and DNS is missing: set AT#CCIOTOPT=0437, reboot
 * - If on Cat-M1 (rat 8) and CCIOTOPT is 0437: restore to 0537, reboot
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
    AZX_LOG_INFO("Current RAT: %d (8=CatM1, 9=NB-IoT)\r\n", current_rat);

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
            while (*p == ' ') p++;      /* FIX: tolerate 0/1 spaces */
            current_opt = (int)strtol(p, NULL, 16);
        }
    }
    AZX_LOG_INFO("Current CCIOTOPT: 0x%04X\r\n", current_opt);

    /* Step 3: Decide */
    if (current_rat == 9 && !dns_present && current_opt != 0x0437)
    {
        AZX_LOG_INFO("NB-IoT without DNS detected, setting CCIOTOPT=0437\r\n");
        if (ati_send_and_wait("AT#CCIOTOPT=0437\r", resp, sizeof(resp), 5000))
        {
            AZX_LOG_INFO("CCIOTOPT set to 0437 OK\r\n");
            need_reboot = TRUE;
        }
    }
    else if (current_rat == 8 && current_opt == 0x0437)
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

/* ---- main network routine ------------------------------------------------- */
BOOLEAN NET_ROUTINE(void)
{
    int ctr;
    BOOLEAN manual_tried = FALSE;

    g_reg_valid = FALSE;

    /* -- 1. Init NET (bounded; old code looped forever) -- */
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

    /* -- 5. Wait for network registration (auto first, manual fallback) -- */
    ctr = 0;
    while (1)
    {
        refresh_reg_status(5000);

        if (g_reg_valid && (g_reg.stat == 1 || g_reg.stat == 5))
            break;

        if (g_reg_valid)
            AZX_LOG_INFO("not connected to network, status:%d\r\n", g_reg.stat);
        else
            AZX_LOG_INFO("no registration status yet\r\n");

        /* FIX: one-shot trigger at >=60 s while still searching (stat 2),
         * instead of requiring ctr to be EXACTLY 60 at the right moment */
        if (!manual_tried && ctr >= 60 && g_reg_valid && g_reg.stat == 2)
        {
            manual_tried = TRUE;
            AZX_LOG_INFO("Stuck at searching, trying manual selection...\r\n");
            if (AUTOMATED_MANUAL_NWSELECTION_ROUTINE())
            {
                AZX_LOG_INFO("letting modem settle...\r\n");
                azx_sleep_ms(5000);
            }
        }

        azx_sleep_ms(1000);
        if (++ctr > 180)   /* total budget: ~3 min auto + scan + manual */
        {
            AZX_LOG_ERROR("Network registration failed after manual attempt\r\n");
            return FALSE;
        }
    }
    AZX_LOG_INFO("Network connection successful!!!\r\n");

    /* -- 6. Activate PDP (FIX: only after registration is confirmed) -- */
    ctr = 0;
    while (M2MB_RESULT_SUCCESS != m2mb_pdp_activate(pdp_handle, PDP_CID,
                                                    PDP_APN, "", "",
                                                    M2MB_PDP_IPV4))
    {
        AZX_LOG_INFO("m2mb_pdp_activate failed\r\n");
        azx_sleep_ms(1000);
        if (++ctr > 120)
        {
            AZX_LOG_ERROR("m2mb_pdp_activate gave up after %d attempts\r\n", ctr);
            break;   /* keep original behavior: continue and let the IP
                      * check / later stages report the real state */
        }
    }
    if (ctr <= 120)
        AZX_LOG_INFO("m2mb_pdp_activate succeeded\r\n");

    {
        UINT8 pdp_stat = 0;
        if (M2MB_RESULT_SUCCESS == m2mb_pdp_get_status(pdp_handle, PDP_CID, &pdp_stat))
        {
            AZX_LOG_INFO("pdp stat:%d\r\n", pdp_stat);
        }
    }

    /* -- 7. Wait for a real (non-zero) IP address -- */
    {
        M2MB_SOCKET_BSD_IN_ADDR addrv4;
        addrv4.s_addr = 0;
        ctr = 0;                       /* FIX: fresh counter */
        while (ctr <= 120)
        {
            if (M2MB_RESULT_SUCCESS == m2mb_pdp_get_my_ip(pdp_handle, PDP_CID,
                                                          M2MB_PDP_IPV4, &addrv4)
                && addrv4.s_addr != 0) /* FIX: 0.0.0.0 keeps waiting */
            {
                UINT8 *ip = (UINT8 *)&addrv4.s_addr;
                AZX_LOG_INFO("=== IP address: %d.%d.%d.%d ===\r\n",
                             ip[0], ip[1], ip[2], ip[3]);
                break;
            }
            azx_sleep_ms(1000);
            ctr++;
        }
        if (addrv4.s_addr == 0)
            AZX_LOG_ERROR("No IPv4 address obtained within timeout\r\n");
    }

    /* -- 8. NB-IoT DNS workaround -- */
    check_nbiot_dns_workaround();

    return TRUE;
}
