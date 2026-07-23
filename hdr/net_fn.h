/* =============================================================================
 * net_fn.h
 * Cellular network + PDP bring-up:
 *   - net/pdp init
 *   - automatic registration wait with manual PLMN-scan fallback
 *   - PDP activation + IP check
 *   - NB-IoT-without-DNS CCIOTOPT workaround
 * ========================================================================== */
#ifndef NET_FN_H
#define NET_FN_H

#include "m2mb_types.h"

/*
 * Full network bring-up routine.
 * Returns TRUE when the module is registered (CEREG stat 1 or 5) and the
 * PDP context activation sequence has been run; FALSE if registration
 * could not be achieved within the overall time budget.
 */
BOOLEAN NET_ROUTINE(void);

#endif /* NET_FN_H */
