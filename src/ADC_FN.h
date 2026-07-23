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

void ati_callback( M2MB_ATI_HANDLE h, M2MB_ATI_EVENTS_E ati_event, UINT16 resp_size, void *resp_struct, void *userdata );
void ADC_ROUTINE();
void ADC_CK();
