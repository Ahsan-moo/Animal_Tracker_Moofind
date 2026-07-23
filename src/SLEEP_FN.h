/* =============================================================================
 * SLEEP_FN.h
 * Power control: alarm-based shutdown cycle, plain shutdown, reboot.
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

/* NOTE: prototype fixed to SET_ALARM (uppercase) -- the old header
 * declared set_alarm(), which matched nothing; the actual function
 * defined in SLEEP_FN.c has always been SET_ALARM(). */
int  SET_ALARM(void);
void ALARM_SHUTDOWN(void);
void SHUTDOWN(void);

/* Reboot the module via m2mb_power_reboot. Falls back to SHUTDOWN() if
 * the reboot request fails, so the alarm cycle always recovers the
 * device. Does not return. */
void REBOOT(void);

#endif /* SLEEP_FN_H */
