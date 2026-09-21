/* =============================================================================
 * sleep_fn.h
 * Power control: alarm-based shutdown cycle, shutdown, reboot.
 *
 * CHANGES IN THIS REVISION:
 *  - Added SET_ALARM_IN() / ALARM_SHUTDOWN_IN() so callers can request a
 *    specific sleep duration instead of only the compiled-in cadence.
 *  - Added ALARM_SHUTDOWN_AFTER_FAILURE() for the "network cycle failed,
 *    sleep long enough to clear the T3402 attach backoff" case.
 * ========================================================================== */
#ifndef SLEEP_FN_H
#define SLEEP_FN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "m2mb_types.h"
#include "m2mb_os_api.h"
#include "m2mb_uart.h"
#include "app_cfg.h"
#include "azx_log.h"
#include "azx_utils.h"
#include <time.h>
#include "m2mb_rtc.h"
#include "m2mb_power.h"
#include "m2mb_net.h"
#include "m2mb_pdp.h"
#include "m2mb_socket.h"
#include "m2mb_lwm2m.h"
#include "m2mb_gnss.h"
#include "m2mb_ati.h"

/* Program the RTC alarm for the normal cadence. Returns -1 on failure. */
int  SET_ALARM(void);

/* Program the RTC alarm <seconds> from now. Returns -1 on failure. */
int  SET_ALARM_IN(UINT32 seconds);

/* Set the normal alarm and power off. Reboots as a last resort if the
 * alarm cannot be programmed, so the device never stays awake unattended. */
void ALARM_SHUTDOWN(void);

/* Set an alarm <seconds> from now and power off. */
void ALARM_SHUTDOWN_IN(UINT32 seconds);

/* Longer sleep used after a failed network cycle, chosen to outlast the
 * network's T3402 attach backoff before the next cold attach attempt. */
void ALARM_SHUTDOWN_AFTER_FAILURE(void);

/* Power off without programming an alarm. Does not return. */
void SHUTDOWN(void);

/* Reboot the module via m2mb_power_reboot. Falls back to SHUTDOWN() if the
 * reboot request fails. Does not return. */
void REBOOT(void);

#endif /* SLEEP_FN_H */
