/* =============================================================================
 * gnss_fn.h
 * GNSS fix acquisition via the modem's location client (LWM2M object 6),
 * XTRA (AGNSS) enablement after first fix, RTC set from GNSS time.
 * ========================================================================== */
#ifndef GNSS_FN_H
#define GNSS_FN_H

#include "m2mb_types.h"

/*
 * Run the GNSS cycle:
 *   1. init GNSS, check XTRA status
 *   2. switch modem priority to GNSS
 *   3. enable location storage (obj 33211), poll obj /6/0/5 for a fix
 *   4. set RTC from GNSS time if RTC still at factory default
 *   5. after first valid fix, request XTRA enable (active next reboot)
 *   6. disable storage, restore WWAN priority
 * Returns TRUE if a GNSS fix was obtained.
 */
BOOLEAN GNSS_ROUTINE(void);

#endif /* GNSS_FN_H */
