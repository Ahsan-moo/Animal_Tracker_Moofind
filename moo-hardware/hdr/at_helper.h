/* =============================================================================
 * at_helper.h
 * Single, shared AT-command send/receive helper over the m2mb ATI interface.
 *
 * FIXES vs original main.c:
 *  - The old code had FOUR copies of the send/poll loop (inline in the DNS
 *    workaround, inline in manual COPS, plus ati_send_and_wait), and every
 *    one of them OVERWROTE the response buffer on each m2mb_ati_rcv_resp()
 *    read. ATI typically delivers the payload line ("+CGCONTRDP: ...") and
 *    the final "OK" in SEPARATE reads, so by the time "OK" arrived the
 *    payload was often already clobbered -> parses randomly failed.
 *    This version ACCUMULATES all chunks into one buffer and stops on
 *    OK / ERROR / timeout.
 * ========================================================================== */
#ifndef AT_HELPER_H
#define AT_HELPER_H

#include "m2mb_types.h"

/*
 * Send one AT command on ATI instance 2 and collect the full response.
 *
 *  cmd        : full command including trailing "\r"
 *  resp_out   : receives the ACCUMULATED response text (may be NULL)
 *  resp_size  : size of resp_out
 *  timeout_ms : overall timeout for the whole response
 *
 * Returns TRUE if "OK" was seen in the accumulated response (and "ERROR"
 * was not seen first).
 */
BOOLEAN ati_send_and_wait(const char *cmd, char *resp_out,
                          UINT16 resp_size, INT32 timeout_ms);

#endif /* AT_HELPER_H */
