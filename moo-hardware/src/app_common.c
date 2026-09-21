/* =============================================================================
 * app_common.c
 * Event-group creation + bounded wait helpers.
 *
 * BUILD NOTE (C++ strictness):
 *   The AppZone toolchain compiles these .c files as C++ (-x c++), and the
 *   SDK defines NULL as ((void*)0). C++ does not implicitly convert void*
 *   to another pointer/handle type, so
 *       M2MB_OS_EV_HANDLE app_evHandle = NULL;
 *   fails with:
 *       error: invalid conversion from 'void*' to 'M2MB_OS_EV_HANDLE'
 *   Every NULL used with M2MB_OS_EV_HANDLE is therefore replaced with a
 *   cast of 0 via the APP_EV_HANDLE_NULL macro below.
 *
 *   If the cast still errors, M2MB_OS_EV_HANDLE is an integer type on your
 *   SDK build -- change the macro to plain 0 (see the comment on it).
 *
 *   NULL is left alone where it is passed to SDK functions expecting a
 *   void* / variadic argument (m2mb_os_ev_setAttrItem below): those are
 *   fine as-is and were compiling before.
 * ========================================================================== */
#include "app_common.h"
#include "azx_log.h"
#include "azx_utils.h"   /* azx_sleep_ms -- this include is missing in one
                          * circulating copy of this file, which fails the
                          * build with "azx_sleep_ms was not declared" */

/* Typed null for the event-group handle. If M2MB_OS_EV_HANDLE turns out to
 * be an integer type rather than a pointer, change this to plain 0. */
#define APP_EV_HANDLE_NULL  ((M2MB_OS_EV_HANDLE)0)

M2MB_OS_EV_HANDLE app_evHandle = APP_EV_HANDLE_NULL;

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

    if (app_evHandle == APP_EV_HANDLE_NULL)
        return FALSE;

    res = m2mb_os_ev_get(app_evHandle, bits, M2MB_OS_EV_GET_ANY_AND_CLEAR,
                         &curBits, M2MB_OS_MS2TICKS(timeout_ms));
    return (res == M2MB_OS_SUCCESS) ? TRUE : FALSE;
}

void set_ev(UINT32 bits)
{
    if (app_evHandle != APP_EV_HANDLE_NULL)
    {
        m2mb_os_ev_set(app_evHandle, bits, M2MB_OS_EV_SET);
    }
}
