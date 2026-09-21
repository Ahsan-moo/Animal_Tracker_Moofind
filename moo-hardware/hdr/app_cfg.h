/*Copyright (C) 2020 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the project root for full license information.     */

#ifndef HDR_APP_CFG_H_
#define HDR_APP_CFG_H_
/**
 * @file app_cfg.h
 * @version 1.0.0
 * @date 10/02/2019
 *
 * @brief Application configuration settings conveniently located here.
 *
 * This file contains macros that a programmer can alter to easily modify the
 * behaviour of the application an **compile** time.
 */

/** @cond DEV*/
#define QUOTE(str) #str
#define EXPAND_AND_QUOTE(str) QUOTE(str)
/** @endcond*/

/*Local basepath for samples that need local files usage*/
#define LOCALPATH "/mod"


/* =============================================================================
 * LOGGING CONFIGURATION                                              FIX [14]
 * =============================================================================
 *
 * WHY THIS BLOCK EXISTS
 *
 * AZX_LOG_INIT() in azx_log.h expands to:
 *
 *     AZX_LOG_CFG_T cfg = { AZX_LOG_LEVEL, LOG_CHANNEL, _LOG_COLOURS };
 *     azx_log_init(&cfg);
 *
 * Neither AZX_LOG_LEVEL nor LOG_CHANNEL was defined in this file before, so
 * both were coming from -D flags in the project build settings -- invisible
 * from the source tree and easy to change by accident. They are made
 * explicit here.
 *
 * THE PROBLEM THIS ADDRESSES
 *
 * The log channel in this project is USB0 (see the "let the log flush over
 * USB0" comment in SLEEP_FN.c). USB0 behaves in three distinct ways
 * depending on how the device is powered:
 *
 *   Laptop, terminal open : VBUS present, host drains the CDC endpoint.
 *                           Logs flow. Application runs normally.
 *
 *   Charger / power bank  : VBUS present, NOTHING drains the endpoint. The
 *                           module sees a plugged cable, so azx_log does not
 *                           report AZX_LOG_USB_CABLE_UNPLUGGED. Writes go to
 *                           an endpoint with no reader.
 *
 *   Lab supply / battery  : no VBUS. azx_log detects the unplugged cable and
 *                           handles it.
 *
 * The middle case is the dangerous one and it is not detectable by the
 * library. The safe answer is not to log at all in builds that will run
 * away from a laptop.
 *
 * HOW TO USE THIS
 *
 *   Bench work, laptop attached:
 *       AZX_LOG_LEVEL = AZX_LOG_LEVEL_INFO   (or DEBUG/TRACE)
 *       LOG_CHANNEL   = AZX_LOG_TO_USB0
 *       and in main.c: DEBUG_BUILD 1, PRODUCTION_BUILD 0
 *
 *   Anything battery / charger / power bank / lab supply powered:
 *       AZX_LOG_LEVEL = AZX_LOG_LEVEL_NONE
 *       and in main.c: DEBUG_BUILD 0, PRODUCTION_BUILD 1
 *
 * main.c also calls azx_log_setLevel(AZX_LOG_LEVEL_NONE) immediately after
 * AZX_LOG_INIT() when PRODUCTION_BUILD is 1. That is belt and braces: this
 * file sets the compile-time default, main.c enforces it at runtime before
 * any other call can block.
 *
 * NOTE ON AZX_LOG_TO_MAIN_UART
 *
 * Do NOT switch LOG_CHANNEL to AZX_LOG_TO_MAIN_UART on this hardware
 * without checking the schematic first. The main UART is ASC0, whose TXD0 /
 * RXD0 lines are wired directly to STM32 pins PA9 and PA10. Nothing on the
 * STM32 side drives those pins today, so it would not cause damage now, but
 * it will collide directly with the planned STM32 <-> modem link.
 *
 * NOTE ON -D FLAGS
 *
 * If the project build settings still pass -DLOG_CHANNEL=... or
 * -DAZX_LOG_LEVEL=..., those definitions arrive BEFORE this header is
 * processed and the #ifndef guards below will leave them in place. Either
 * remove the -D flags from the project settings, or replace the #ifndef
 * guards with #undef followed by #define to force the values here.
 * ========================================================================== */

/* Bring the enum names into scope for the macros below. */
#include "azx_log.h"

/* ---------------------------------------------------------------------------
 * Output channel. AZX_LOG_TO_USB0 is correct for bench work with a laptop.
 * ------------------------------------------------------------------------- */
#ifndef LOG_CHANNEL
#define LOG_CHANNEL         AZX_LOG_TO_USB0
#endif

/* ---------------------------------------------------------------------------
 * Verbosity.
 *
 *   AZX_LOG_LEVEL_TRACE    everything
 *   AZX_LOG_LEVEL_DEBUG    most messages
 *   AZX_LOG_LEVEL_INFO     message text only, no prefixes   <-- bench default
 *   AZX_LOG_LEVEL_WARN     warnings and worse
 *   AZX_LOG_LEVEL_ERROR    errors and worse
 *   AZX_LOG_LEVEL_NONE     nothing at all                   <-- field default
 * ------------------------------------------------------------------------- */
#ifndef AZX_LOG_LEVEL
#define AZX_LOG_LEVEL       AZX_LOG_LEVEL_INFO
#endif

/* ---------------------------------------------------------------------------
 * Colour escape sequences. Useful in a terminal, noise in a captured file.
 * ------------------------------------------------------------------------- */
#ifndef AZX_LOG_ENABLE_COLOURS
#define AZX_LOG_ENABLE_COLOURS   0
#endif

#endif /* HDR_APP_CFG_H_ */
