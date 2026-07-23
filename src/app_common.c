/* =============================================================================
 * app_common.c
 * Event-group creation + bounded wait helpers.
 * ========================================================================== */
#include "app_common.h"
#include "azx_log.h"
#include "azx_utils.h"   /* azx_sleep_ms */

M2MB_OS_EV_HANDLE app_evHandle = NULL;

BOOLEAN app_ev_init(void)
{
    M2MB_OS_RESULT_E osRes;
    M2MB_OS_EV_ATTR_HANDLE evAttrHandle;
    int attempts = 0;

    osRes = m2mb_os_ev_setAttrItem(&evAttrHandle,
              CMDS_ARGS(M2MB_OS_EV_SEL_CMD_CREATE_ATTR, NULL,
                        M2MB_OS_EV_SEL_CMD_NAME, "app_ev"));
    if (osRes != M2MB_OS_SUCCESS)
    {
        AZX_LOG_ERROR("ev attr create failed\r\n");
        return FALSE;
    }

    /* FIX: the old loop retried forever; a persistent OS failure would hang
     * the device with the radio on. Bound it and report the failure up. */
    while (M2MB_OS_SUCCESS != (osRes = m2mb_os_ev_init(&app_evHandle, &evAttrHandle)))
    {
        AZX_LOG_ERROR("m2mb_os_ev_init failed (res=%d)\r\n", osRes);
        if (++attempts >= 10)
        {
            m2mb_os_ev_setAttrItem(&evAttrHandle, M2MB_OS_EV_SEL_CMD_DEL_ATTR, NULL);
            return FALSE;
        }
        azx_sleep_ms(500);
    }

    AZX_LOG_INFO("m2mb_os_ev_init success\r\n");
    return TRUE;
}

BOOLEAN wait_ev(UINT32 bits, UINT32 timeout_ms)
{
    UINT32 curBits = 0;
    M2MB_OS_RESULT_E res;

    if (app_evHandle == NULL)
        return FALSE;

    res = m2mb_os_ev_get(app_evHandle, bits, M2MB_OS_EV_GET_ANY_AND_CLEAR,
                         &curBits, M2MB_OS_MS2TICKS(timeout_ms));
    return (res == M2MB_OS_SUCCESS) ? TRUE : FALSE;
}

void set_ev(UINT32 bits)
{
    if (app_evHandle != NULL)
    {
        m2mb_os_ev_set(app_evHandle, bits, M2MB_OS_EV_SET);
    }
}
