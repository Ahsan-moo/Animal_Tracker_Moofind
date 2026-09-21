/* =============================================================================
 * net_fn.h
 * Cellular network + PDP bring-up:
 *   - net/pdp init
 *   - automatic registration wait with manual PLMN-scan fallback
 *   - PDP activation + IP check
 *   - NB-IoT-without-DNS CCIOTOPT workaround
 *
 * CHANGES IN THIS REVISION:
 *  - NET_ROUTINE's contract corrected: it now returns the real outcome
 *    (registered AND a non-zero IP), not an unconditional TRUE.
 *  - Added NET_WAIT_REGISTERED() so the caller can confirm the modem is
 *    back on the network after GNSS has released the radio.
 * ========================================================================== */
#ifndef NET_FN_H
#define NET_FN_H

#include "m2mb_types.h"

/*
 * Full network bring-up routine.
 *
 * Returns TRUE only when the module is registered (stat 1 or 5) AND a
 * non-zero IPv4 address has been obtained -- i.e. the network is genuinely
 * usable. Returns FALSE on registration denial (stat 3, terminal), on
 * registration timeout, or when no address was assigned.
 *
 * NOTE: the previous revision ended in an unconditional `return TRUE;`,
 * so callers' failure branches were dead code. Callers that were written
 * against that behavior will now actually see failures.
 *
 * The registration wait is sized from the network's own T3402 attach
 * backoff (read from AT#RFSTS, typically 720 s) so the routine does not
 * give up while the modem is still in a mandated backoff period.
 */
BOOLEAN NET_ROUTINE(void);

/*
 * Poll registration status until the modem reads stat 1 (home) or 5
 * (roaming), up to timeout_ms.
 *
 * Intended for use after GNSS_ROUTINE, which switches the module to GNSS
 * priority and takes the cellular link down for the duration of the fix
 * attempt. A fixed settle delay is not adequate there: a warm fix releases
 * the radio in ~40 s while a timeout holds it for the full GNSS budget.
 * Scale the timeout with GNSS_LAST_HELD_RADIO_MS() / GNSS_LAST_TIMED_OUT().
 *
 * Returns TRUE once registered, FALSE on denial (stat 3) or timeout.
 * Returns FALSE immediately if NET_ROUTINE has not yet initialised the
 * network handle.
 */
BOOLEAN NET_WAIT_REGISTERED(UINT32 timeout_ms);

#endif /* NET_FN_H */
