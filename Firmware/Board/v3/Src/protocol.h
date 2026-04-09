#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* ── HID Command IDs (Feature Report 0x03, host → device) ───────────────────
 *
 *   Legacy (kept for backwards compat with old GUI):
 *     0x01  SET_AXIS_STATE   value = state id (float cast)
 *     0x02  CLEAR_ERRORS     —
 *     0x05  ENTER_DFU        —
 *     0x06  SET_INPUT_POS    value = position (counts)
 *     0x07  SET_CURRENT_LIM  value = limit (A)    [deprecated: use 0x10]
 *
 *   Generic configuration:
 *     0x10  SET_CONFIG       param_id + value
 *     0x11  GET_CONFIG       param_id  → responds with Report 0x04
 *     0x12  CALL             param_id = call function id (value unused)
 *     0x13  SAVE_CONFIG      — (deferred to main loop)
 *     0x14  REBOOT           —
 * ─────────────────────────────────────────────────────────────────────────── */
#define HID_CMD_SET_AXIS_STATE   0x01U
#define HID_CMD_CLEAR_ERRORS     0x02U
#define HID_CMD_ENTER_DFU        0x05U
#define HID_CMD_SET_INPUT_POS    0x06U
#define HID_CMD_SET_CURRENT_LIM  0x07U   /* deprecated */

#define HID_CMD_SET_CONFIG       0x10U
#define HID_CMD_GET_CONFIG       0x11U
#define HID_CMD_CALL             0x12U
#define HID_CMD_SAVE_CONFIG      0x13U
#define HID_CMD_REBOOT           0x14U

/* ── CALL function IDs (used as param_id with HID_CMD_CALL) ─────────────────
 *   The CALL command sets a flag read by the main loop; it never executes
 *   directly from the ISR to avoid re-entrancy and stack issues.
 * ─────────────────────────────────────────────────────────────────────────── */
#define CALL_MOTOR_CALIBRATION      1U
#define CALL_ENCODER_CALIBRATION    2U
#define CALL_ENCODER_INDEX_SEARCH   3U
#define CALL_ENTER_CLOSED_LOOP      4U
#define CALL_CLEAR_ERRORS           5U
#define CALL_SET_IDLE               6U

/* ── Command payload (Feature Report payload, 8 bytes) ───────────────────────
 * Report 0x03 layout: [report_id(1)] [HID_Command_t(8)]
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  cmd_id;     /* HID_CMD_*                                          */
    uint16_t param_id;   /* Config param ID (CFG_*) or CALL_* when cmd=CALL   */
    float    value;      /* Payload — 0.0 when unused                          */
    uint8_t  reserved;
} HID_Command_t;         /* 8 bytes */

/* ── Config Response (Input Report 0x04, 6 bytes payload) ────────────────────
 * Sent by firmware in response to HID_CMD_GET_CONFIG.
 * Report layout: [report_id(1)] [HID_ConfigResponse_t(6)]
 * ─────────────────────────────────────────────────────────────────────────── */
#define HID_REPORT_ID_CONFIG_RESP  0x04U

typedef struct __attribute__((packed)) {
    uint8_t  report_id;  /* Always HID_REPORT_ID_CONFIG_RESP (0x04)           */
    uint16_t param_id;   /* Echoes the requested param_id                     */
    float    value;      /* Current value                                     */
} HID_ConfigResponse_t;  /* 7 bytes total */

/* ── Deferred action flags (set in ISR, cleared in main loop) ────────────────
 * Using volatile ensures the compiler does not cache the read.
 * On Cortex-M4, single-word reads/writes are atomic.
 * ─────────────────────────────────────────────────────────────────────────── */
typedef enum {
    PENDING_CALL_NONE           = 0,
    PENDING_CALL_MOTOR_CAL      = CALL_MOTOR_CALIBRATION,
    PENDING_CALL_ENCODER_CAL    = CALL_ENCODER_CALIBRATION,
    PENDING_CALL_ENCODER_INDEX  = CALL_ENCODER_INDEX_SEARCH,
    PENDING_CALL_CLOSED_LOOP    = CALL_ENTER_CLOSED_LOOP,
    PENDING_CALL_CLEAR_ERRORS   = CALL_CLEAR_ERRORS,
    PENDING_CALL_SET_IDLE       = CALL_SET_IDLE,
} PendingCall_t;

extern volatile PendingCall_t g_pending_call;  /* read/written from ISR+loop  */
extern volatile bool          g_pending_save;  /* flash save requested        */

/* ── Application callback table ─────────────────────────────────────────────
 * Register once at startup (protocol_init).  The protocol module never calls
 * HAL or RTOS directly — all hardware interaction goes through these.
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct {
    void (*set_axis_state)(uint8_t state);   /* Change requested_state        */
    void (*set_input_pos) (float pos);       /* Write input_pos to controller */
    void (*clear_errors)  (void);            /* Clear all axis/motor/enc errors*/
    void (*enter_dfu)     (void);            /* Reboot into DFU mode          */
} ProtocolCallbacks_t;

/* ── Public API ────────────────────────────────────────────────────────────── */

/* Call once at startup, before USB initialisation. */
void protocol_init(const ProtocolCallbacks_t *callbacks);

/* Entry point called from USB ISR (via HID_ODrive_ProcessCommand).
 * Must be non-blocking.  Deferred work is signalled via g_pending_*.         */
void protocol_dispatch(const HID_Command_t *cmd);

/* Call from main loop / RTOS task.  Executes deferred calls and flash saves. */
void protocol_process_pending(void);

/* Send a config response over HID (called inside protocol_dispatch).         */
bool protocol_send_config_response(uint16_t param_id, float value);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
