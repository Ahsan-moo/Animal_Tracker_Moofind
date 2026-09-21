/* =============================================================================
 * app_common.h
 * Shared application-wide definitions: event bits, tick conversion,
 * event-group helpers used by every module.
 *
 * FIXES vs original main.c:
 *  - One event bit PER response type (old code shared EV_MOO_BIT for ~8
 *    different responses, so any unsolicited indication could satisfy the
 *    wrong wait and pair a request with the wrong response).
 *  - wait_ev() now takes milliseconds and converts via M2MB_OS_MS2TICKS
 *    instead of assuming 1 tick == 1 ms.
 *  - No more M2MB_OS_WAIT_FOREVER anywhere: every wait is bounded, so a
 *    missed callback can no longer hang the device and drain the battery.
 * ========================================================================== */
#ifndef APP_COMMON_H
#define APP_COMMON_H

#include "m2mb_types.h"
#include "m2mb_os_api.h"

/* ---- Event bits (one per response type) ---------------------------------- */
#define EV_NET_REG_BIT      ((UINT32)0x0001)  /* reg status response          */
#define EV_NET_SIG_BIT      ((UINT32)0x0002)  /* signal info response         */
#define EV_NET_OP_BIT       ((UINT32)0x0004)  /* current operator response    */
#define EV_NW_LIST_BIT      ((UINT32)0x0008)  /* available NW list (kept 0x8) */
#define EV_NET_MODE_BIT     ((UINT32)0x0010)  /* mode preference response     */
#define EV_NET_BER_BIT      ((UINT32)0x0020)  /* BER response                 */
#define EV_LWM2M_STAT_BIT   ((UINT32)0x0040)  /* LWM2M get_stat response      */
#define EV_LWM2M_READ_BIT   ((UINT32)0x0080)  /* LWM2M read response          */
#define EV_LWM2M_WRITE_BIT  ((UINT32)0x0100)  /* LWM2M write response         */
#define EV_GNSS_POS_BIT     ((UINT32)0x0200)  /* GNSS position report         */

/* ---- Tick conversion ----------------------------------------------------- */
/* The SDK normally provides M2MB_OS_MS2TICKS. If this firmware's headers do
 * not, fall back to 1 tick == 1 ms (what the old code silently assumed). */
#ifndef M2MB_OS_MS2TICKS
#define M2MB_OS_MS2TICKS(ms) (ms)
#endif

/* Single application event group, created once in M2MB_main() */
extern M2MB_OS_EV_HANDLE app_evHandle;

/* Create the event group. Returns TRUE on success. */
BOOLEAN app_ev_init(void);

/* Wait for (any of) the given bit(s), clearing them on receipt.
 * timeout_ms is in milliseconds. Returns TRUE if the bit arrived in time. */
BOOLEAN wait_ev(UINT32 bits, UINT32 timeout_ms);

/* Set bit(s) from a callback context. */
void set_ev(UINT32 bits);

#endif /* APP_COMMON_H */
