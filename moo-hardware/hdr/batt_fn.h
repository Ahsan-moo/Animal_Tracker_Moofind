/* =============================================================================
 * batt_fn.h
 * Battery voltage read (AT#CBC), percent estimation, and LWM2M battery
 * resource update (/3/0/9).
 * ========================================================================== */
#ifndef BATT_FN_H
#define BATT_FN_H

#include "m2mb_types.h"

/* Read battery voltage, compute percent, push to LWM2M /3/0/9.
 * Returns TRUE on success. */
BOOLEAN BATT_STAT_ROUTINE(void);

#endif /* BATT_FN_H */
