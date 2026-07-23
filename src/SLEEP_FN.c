/* =============================================================================
 * SLEEP_FN.c
 * Alarm-based shutdown cycle, plain shutdown, and reboot.
 *
 * Change log:
 *  - Added REBOOT(): reboots via m2mb_power_reboot, mirroring SHUTDOWN's
 *    power-init style; falls back to SHUTDOWN() if the reboot request
 *    fails so the alarm cycle always recovers the device.
 *  - No changes to SET_ALARM / ALARM_SHUTDOWN / SHUTDOWN behavior.
 * ========================================================================== */
#include "SLEEP_FN.h"
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

#define TIME_WAKEUP    60000        /* in seconds */
#define SLEEP_INTERVAL 0.1*60*60    /* 360 s = 6 min between wake cycles */

void *sleep_userdata = NULL;
INT32 rtcfd;

int SET_ALARM(void)
{
  M2MB_RTC_TIMEVAL_T time_val;
  struct tm * ptm;
  time_t time_val_t;
  M2MB_RTC_TIME_T rtc_time;
  INT32 ret = 0;

  rtcfd = m2mb_rtc_open("/dev/rtc0", 0);
  if (rtcfd != -1)
  {
      AZX_LOG_INFO("RTC opened\r\n");
  }
  else
  {
      AZX_LOG_INFO("Cannot open RTC!");
      return -1;
  }

  ret  = m2mb_rtc_ioctl(rtcfd, M2MB_RTC_IOCTL_GET_TIMEVAL, &time_val);
  ret |= m2mb_rtc_ioctl(rtcfd, M2MB_RTC_IOCTL_GET_SYSTEM_TIME, &rtc_time);
  AZX_LOG_INFO("Module system time is: %d-%02d-%02d, %02d:%02d:%02d\r\n",
               rtc_time.year, rtc_time.mon, rtc_time.day,
               rtc_time.hour, rtc_time.min, rtc_time.sec);

  time_val.sec += SLEEP_INTERVAL;
  time_val_t = time_val.sec;
  ptm = gmtime(&time_val_t);
  if (ptm != NULL)
  {
      rtc_time.year = ptm->tm_year + 1900;
      rtc_time.mon  = ptm->tm_mon + 1;
      rtc_time.day  = ptm->tm_mday;
      rtc_time.hour = ptm->tm_hour;
      rtc_time.min  = ptm->tm_min;
      rtc_time.sec  = ptm->tm_sec;
      AZX_LOG_INFO("Alarm will be set at: %d-%02d-%02d, %02d:%02d:%02d\r\n",
                   rtc_time.year, rtc_time.mon, rtc_time.day,
                   rtc_time.hour, rtc_time.min, rtc_time.sec);
      ret |= m2mb_rtc_ioctl(rtcfd, M2MB_RTC_IOCTL_SET_ALARM_TIME, &rtc_time, 0x01);
  }
  else
  {
      AZX_LOG_INFO("Impossible to set an alarm\r\n");
      ret = -1;
  }

  m2mb_rtc_close(rtcfd);
  return ret;
}

void ALARM_SHUTDOWN(void)
{
    if (SET_ALARM() != -1)
    {
        SHUTDOWN();
    }
    else
    {
        AZX_LOG_INFO("cannot set alarm\r\n");
    }
}

void SHUTDOWN(void)
{
    M2MB_POWER_HANDLE power_handle = NULL;
    if (M2MB_RESULT_SUCCESS == m2mb_power_init(&power_handle,
                                               (m2mb_power_ind_callback) NULL,
                                               sleep_userdata))
    {
        AZX_LOG_INFO("Power off module\r\n");
        m2mb_power_shutdown(power_handle);
    }
    else
    {
        AZX_LOG_ERROR("Cannot init power apis \r\n!");
        return;
    }
}

void REBOOT(void)
{
    M2MB_POWER_HANDLE power_handle = NULL;

    AZX_LOG_INFO("REBOOT: rebooting module...\r\n");
    azx_sleep_ms(500);   /* let the log flush before the radio goes down */

    if (M2MB_RESULT_SUCCESS == m2mb_power_init(&power_handle,
                                               (m2mb_power_ind_callback) NULL,
                                               sleep_userdata))
    {
        m2mb_power_reboot(power_handle);
    }

    /* Should never get here. If the reboot request failed, shut down so
     * the alarm cycle recovers the device instead of hanging awake and
     * draining the battery. */
    AZX_LOG_ERROR("REBOOT: m2mb_power_reboot failed, shutting down instead\r\n");
    SHUTDOWN();
}
