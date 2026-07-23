/* =============================================================================
 * lwm2m_fn.h
 * LWM2M client control: init, enable/registration cycle, and small
 * read/write helpers used by the GNSS module.
 * ========================================================================== */
#ifndef LWM2M_FN_H
#define LWM2M_FN_H

#include "m2mb_types.h"
#include "m2mb_lwm2m.h"

/* Initialize the LWM2M client handle. Returns TRUE on success. */
BOOLEAN LWM2M_initializer(void);

/* Enable client, wait for registration (clStatus == 4), let the agent
 * report, then disable. Returns TRUE if registration was observed. */
BOOLEAN LWM2M_ROUTINE(void);

/* Write a single UINT32 value to a LWM2M resource and wait for the
 * write acknowledgment (bounded). Returns TRUE on success. */
BOOLEAN lwm2m_write_u32(M2MB_LWM2M_OBJ_URI_T *uri, UINT32 value);

/* Read a resource into out_buf (out_len bytes) and wait for the read
 * response (bounded). Returns TRUE if the response arrived in time. */
BOOLEAN lwm2m_read_raw(M2MB_LWM2M_OBJ_URI_T *uri, void *out_buf,
                       UINT16 out_len, UINT32 timeout_ms);

#endif /* LWM2M_FN_H */
