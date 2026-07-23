#include "ADC_FN.h"
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

M2MB_ATI_HANDLE ati_handle;

void *userdata_adc = NULL;

void ati_callback( M2MB_ATI_HANDLE h, M2MB_ATI_EVENTS_E ati_event, UINT16 resp_size, void *resp_struct, void *userdata )
{
  switch(ati_event)
  {
  case M2MB_RX_DATA_EVT:
  {

  }
  }
}

void ADC_ROUTINE()
{
	while (!(M2MB_RESULT_SUCCESS == m2mb_ati_init(&ati_handle, 2, ati_callback, userdata_adc)))
	{
		AZX_LOG_INFO("m2mb_ati_init failed \r\n");
		azx_sleep_ms(2000);
	}
	AZX_LOG_INFO("m2mb_ati_init success \r\n");

	char cmd_str[30];
	char resp_str[50];
	int adc_previous;
	int adc_read;
	int adc_read_round;

	sprintf(cmd_str, "AT#LWM2MR=0,3,0,8,0\r");
	if (M2MB_RESULT_SUCCESS == m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str)))
	{
		AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
		while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50))
		{
			AZX_LOG_INFO("no response \r\n");
			azx_sleep_ms(500);
		}
		char resp_sub[3];
		strncpy(resp_sub, resp_str + 31, 2);
		resp_sub[2] = '\0';
		adc_previous = atoi(resp_sub);
		//AZX_LOG_INFO("saved value: %d str:%s \n", adc_previous, resp_sub);
	}

	sprintf(cmd_str, "AT#GPIO=2,1,1\r");
	if (M2MB_RESULT_SUCCESS == m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str))) {
		AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
		while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50))
		{
			AZX_LOG_INFO("no response \r\n");
			azx_sleep_ms(500);
		}
		//AZX_LOG_INFO("response: %s \n", resp_str);
	}

	azx_sleep_ms(1000);

	sprintf(cmd_str, "AT#ADC=1,2,0\r");
	if (M2MB_RESULT_SUCCESS == m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str)))
	{
		AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
		while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50))
		{
			AZX_LOG_INFO("no response \r\n");
			azx_sleep_ms(500);
		}
		char resp_sub[5];
		strncpy(resp_sub, resp_str + 21, 4);
		resp_sub[4] = '\0';
		float num1 = atoi(resp_sub);
		num1 = (((num1*2.8) - 3600)/600)*100;
		if(num1 < 0)
		{
			num1 =  0;
		}
		if(num1 > 100)
		{
			num1 =  100;
		}
		adc_read = (int) num1;
		AZX_LOG_INFO("response: %f %d str:%s \r\n", num1, adc_read, resp_sub);
	}

	adc_read_round = adc_read / 10;
	adc_read_round = adc_read_round * 10;
	if ((adc_read - adc_read_round) > 4)
	{
		adc_read_round += 10;
	}

	int dif;
	if(adc_read>adc_previous)
	{
		dif = adc_read - adc_previous;
	}
	else
	{
		dif = adc_previous - adc_read;
	}

	if(dif > 5)
	{
		sprintf(cmd_str, "AT#LWM2MSET=0,3,0,9,0,%d\r", adc_read_round);
		if (M2MB_RESULT_SUCCESS == m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str)))
		{
			AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
			while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50))
			{
				AZX_LOG_INFO("no response \r\n");
				azx_sleep_ms(500);
			}
			//AZX_LOG_INFO("response: %s \r\n", resp_str);
		}

		sprintf(cmd_str, "AT#LWM2MSET=0,3,0,8,0,%d\r", adc_read);
		if (M2MB_RESULT_SUCCESS == m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str))) {
			AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
			while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50))
			{
				AZX_LOG_INFO("no response \r\n");
				azx_sleep_ms(500);
			}
			//AZX_LOG_INFO("response: %s \n", resp_str);
		}
	}
	AZX_LOG_INFO("prev: %d now: %d round: %d \r\n", adc_previous, adc_read, adc_read_round);

	sprintf(cmd_str, "AT#GPIO=2,0,1\r");
	if (M2MB_RESULT_SUCCESS == m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str))) {
	AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
	while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50))
	{
		AZX_LOG_INFO("no response \r\n");
		azx_sleep_ms(500);
	}
	//AZX_LOG_INFO("response: %s \n", resp_str);
	}

	m2mb_ati_deinit(ati_handle);
}

void ADC_CK()
{
	while (!(M2MB_RESULT_SUCCESS
			== m2mb_ati_init(&ati_handle, 2, ati_callback, userdata_adc))) {
		AZX_LOG_INFO("m2mb_ati_init failed \r\n");
		azx_sleep_ms(2000);
	}
	AZX_LOG_INFO("m2mb_ati_init success \r\n");

	char cmd_str[30];
	char resp_str[50];
	int adc_previous;
	int adc_read;
	int adc_read_round;

	sprintf(cmd_str, "AT#LWM2MR=0,3,0,8,0\r");
	if (M2MB_RESULT_SUCCESS
			== m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str))) {
		AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
		while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50)) {
			AZX_LOG_INFO("no response \r\n");
			azx_sleep_ms(500);
		}
		char resp_sub[3];
		strncpy(resp_sub, resp_str + 31, 2);
		resp_sub[2] = '\0';
		adc_previous = atoi(resp_sub);
		//AZX_LOG_INFO("saved value: %d str:%s \n", adc_previous, resp_sub);
	}

	sprintf(cmd_str, "AT#GPIO=2,1,1\r");
	if (M2MB_RESULT_SUCCESS
			== m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str))) {
		AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
		while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50)) {
			AZX_LOG_INFO("no response \r\n");
			azx_sleep_ms(500);
		}
		//AZX_LOG_INFO("response: %s \n", resp_str);
	}

	azx_sleep_ms(1000);

	sprintf(cmd_str, "AT#ADC=1,2,0\r");
	if (M2MB_RESULT_SUCCESS
			== m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str))) {
		AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
		while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50)) {
			AZX_LOG_INFO("no response \r\n");
			azx_sleep_ms(500);
		}
		char resp_sub[5];
		strncpy(resp_sub, resp_str + 21, 4);
		resp_sub[4] = '\0';
		float num1 = atoi(resp_sub);
		num1 = (((num1 * 2.8) - 3600) / 600) * 100;
		if (num1 < 0) {
			num1 = 0;
		}
		if (num1 > 100) {
			num1 = 100;
		}
		adc_read = (int) num1;
		AZX_LOG_INFO("response: %f %d str:%s \r\n", num1, adc_read, resp_sub);
	}

	adc_read_round = adc_read / 10;
	adc_read_round = adc_read_round * 10;
	if ((adc_read - adc_read_round) > 4) {
		adc_read_round += 10;
	}

	int dif;
	if (adc_read > adc_previous) {
		dif = adc_read - adc_previous;
	} else {
		dif = adc_previous - adc_read;
	}

	if (dif > 5) {
		sprintf(cmd_str, "AT#LWM2MSET=0,3,0,9,0,%d\r", adc_read_round);
		if (M2MB_RESULT_SUCCESS
				== m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str))) {
			AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
			while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50)) {
				AZX_LOG_INFO("no response \r\n");
				azx_sleep_ms(500);
			}
			//AZX_LOG_INFO("response: %s \r\n", resp_str);
		}

		sprintf(cmd_str, "AT#LWM2MSET=0,3,0,8,0,%d\r", adc_read);
		if (M2MB_RESULT_SUCCESS
				== m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str))) {
			AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
			while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50)) {
				AZX_LOG_INFO("no response \r\n");
				azx_sleep_ms(500);
			}
			//AZX_LOG_INFO("response: %s \n", resp_str);
		}
	}
	AZX_LOG_INFO("prev: %d now: %d round: %d \r\n", adc_previous, adc_read,
			adc_read_round);

	sprintf(cmd_str, "AT#GPIO=2,0,1\r");
	if (M2MB_RESULT_SUCCESS
			== m2mb_ati_send_cmd(ati_handle, cmd_str, strlen(cmd_str))) {
		AZX_LOG_INFO("m2mb_ati_send_cmd success \r\n");
		while (0 == m2mb_ati_rcv_resp(ati_handle, resp_str, 50)) {
			AZX_LOG_INFO("no response \r\n");
			azx_sleep_ms(500);
		}
		//AZX_LOG_INFO("response: %s \n", resp_str);
	}

	m2mb_ati_deinit(ati_handle);
}
