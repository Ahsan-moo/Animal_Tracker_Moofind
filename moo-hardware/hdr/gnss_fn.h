/* =============================================================================
 * gnss_fn.h
 * GNSS fix acquisition via the modem's location client (LWM2M object 6),
 * XTRA (AGNSS) enablement, RTC set from GNSS time.
 *
 * CHANGES IN THIS REVISION:
 *  - Added GNSS_LAST_HELD_RADIO_MS() and GNSS_LAST_TIMED_OUT() so the
 *    caller can tell how long the cellular link was down and size its
 *    re-attach wait accordingly.
 *  - Fix budget restored to 5 min after a field cold start landed at poll
 *    13 (see gnss_fn.c).
 *  - XTRA enable no longer gated behind a successful fix (deadlock).
 *
 * BUILD NOTE: delete any uppercase GNSS_FN.h from the project. It is an
 * obsolete stub with the GNSS_ROUTINE declaration commented out, and on a
 * case-insensitive filesystem it shadows this file.
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
 *   5. request XTRA enable if not already active (takes effect next boot)
 *   6. disable storage, restore WWAN priority
 *
 * Returns TRUE if a GNSS fix was obtained.
 *
 * IMPORTANT: on this shared-RF part, steps 2-6 hold GNSS priority, which
 * takes the CELLULAR LINK DOWN for the whole of that window. Callers must
 * confirm the modem has re-attached (NET_WAIT_REGISTERED) before starting
 * any network-dependent stage.
 */
BOOLEAN GNSS_ROUTINE(void);

/*
 * Approximate time, in milliseconds, that the last GNSS_ROUTINE call held
 * GNSS priority -- i.e. how long the cellular link was unavailable.
 * Valid only after GNSS_ROUTINE has returned. Zero if it never took the
 * radio (e.g. GNSS init failed).
 */
UINT32 GNSS_LAST_HELD_RADIO_MS(void);

/*
 * TRUE if the last GNSS_ROUTINE call ended by exhausting its fix budget
 * rather than by obtaining a fix. A timeout means the radio was held for
 * the full budget, so the modem needs correspondingly longer to re-attach.
 * Valid only after GNSS_ROUTINE has returned.
 */
BOOLEAN GNSS_LAST_TIMED_OUT(void);

#endif /* GNSS_FN_H */
