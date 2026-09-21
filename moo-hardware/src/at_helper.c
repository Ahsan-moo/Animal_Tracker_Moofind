/* =============================================================================
 * at_helper.c
 * ========================================================================== */
#include <string.h>
#include "at_helper.h"
#include "m2mb_os_api.h"
#include "m2mb_ati.h"      /* if your SDK names this header differently
                            * (e.g. m2mb_atp.h exposing the ATI calls),
                            * adjust this single include */
#include "azx_log.h"
#include "azx_utils.h"

#define ATI_INSTANCE      2
#define ATI_POLL_MS       200
#define ATI_ACCUM_SIZE    512

BOOLEAN ati_send_and_wait(const char *cmd, char *resp_out,
                          UINT16 resp_size, INT32 timeout_ms)
{
    M2MB_ATI_HANDLE ati_h = NULL;
    BOOLEAN ok = FALSE;
    char accum[ATI_ACCUM_SIZE];
    char chunk[128];
    UINT16 used = 0;
    INT32 elapsed = 0;

    accum[0] = '\0';
    if (resp_out != NULL && resp_size > 0)
        resp_out[0] = '\0';

    if (cmd == NULL)
        return FALSE;

    if (M2MB_RESULT_SUCCESS != m2mb_ati_init(&ati_h, ATI_INSTANCE, NULL, NULL))
    {
        AZX_LOG_ERROR("ati_send_and_wait: ATI init failed\r\n");
        return FALSE;
    }

    if (M2MB_RESULT_SUCCESS == m2mb_ati_send_cmd(ati_h, (char *)cmd, strlen(cmd)))
    {
        while (elapsed < timeout_ms)
        {
            INT32 n = m2mb_ati_rcv_resp(ati_h, chunk, sizeof(chunk) - 1);
            if (n > 0)
            {
                if (n > (INT32)(sizeof(chunk) - 1))
                    n = (INT32)(sizeof(chunk) - 1);
                chunk[n] = '\0';

                /* FIX: append instead of overwrite */
                if (used + (UINT16)n < (ATI_ACCUM_SIZE - 1))
                {
                    memcpy(&accum[used], chunk, (size_t)n);
                    used += (UINT16)n;
                    accum[used] = '\0';
                }

                if (strstr(accum, "ERROR") != NULL)
                {
                    ok = FALSE;
                    break;
                }
                if (strstr(accum, "OK") != NULL)
                {
                    ok = TRUE;
                    break;
                }
                /* payload chunk received but final result code not yet:
                 * keep reading without sleeping */
                continue;
            }

            azx_sleep_ms(ATI_POLL_MS);
            elapsed += ATI_POLL_MS;
        }

        if (elapsed >= timeout_ms)
        {
            AZX_LOG_ERROR("ati_send_and_wait: timeout for '%s'\r\n", cmd);
        }
    }
    else
    {
        AZX_LOG_ERROR("ati_send_and_wait: send failed for '%s'\r\n", cmd);
    }

    m2mb_ati_deinit(ati_h);

    if (resp_out != NULL && resp_size > 0)
    {
        strncpy(resp_out, accum, resp_size - 1);
        resp_out[resp_size - 1] = '\0';
    }

    return ok;
}
