#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* ── HID Command IDs ─────────────────────────────────────────────────────────
 *   0x01  SET_AXIS_STATE   value = state id
 *   0x02  CLEAR_ERRORS
 *   0x05  ENTER_DFU
 *   0x06  SET_INPUT_POS    value = position
 *   0x07  SET_CURRENT_LIM  [deprecated → use 0x10]
 *   0x10  SET_CONFIG       param_id + value
 *   0x11  GET_CONFIG       param_id → responds with Report 0x04
 *   0x12  CALL             param_id = call function id
 *   0x13  SAVE_CONFIG
 *   0x14  REBOOT
 * ─────────────────────────────────────────────────────────────────────────── */
#define HID_CMD_SET_AXIS_STATE   0x01U
#define HID_CMD_CLEAR_ERRORS     0x02U
#define HID_CMD_ENTER_DFU        0x05U
#define HID_CMD_SET_INPUT_POS    0x06U
#define HID_CMD_SET_CURRENT_LIM  0x07U

#define HID_CMD_SET_CONFIG       0x10U
#define HID_CMD_GET_CONFIG       0x11U
#define HID_CMD_CALL             0x12U
#define HID_CMD_SAVE_CONFIG      0x13U
#define HID_CMD_REBOOT           0x14U

/* ── CALL function IDs ───────────────────────────────────────────────────────
 *   Used as param_id with HID_CMD_CALL.
 * ─────────────────────────────────────────────────────────────────────────── */
#define CALL_MOTOR_CALIBRATION      1U
#define CALL_ENCODER_CALIBRATION    2U
#define CALL_ENCODER_INDEX_SEARCH   3U
#define CALL_ENTER_CLOSED_LOOP      4U
#define CALL_CLEAR_ERRORS           5U
#define CALL_SET_IDLE               6U
#define CALL_APPLY_CONFIG           7U  /* apply g_config to ODrive live objects */
#define CALL_SET_HOME               8U  /* capture current pos as position zero  */
#define CALL_FFB_TEST_ON            9U  /* enable FFB actuators (test without game) */
#define CALL_FFB_TEST_OFF           10U /* disable FFB actuators / stop test mode  */

/* ── Command payload (Feature Report payload, 8 bytes) ─────────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  cmd_id;
    uint16_t param_id;
    float    value;
    uint8_t  reserved;
} HID_Command_t;

/* ── Config Response (Input Report 0x04) ────────────────────────────────────── */
#define HID_REPORT_ID_CONFIG_RESP  0x04U

/* ── Deferred action flags ──────────────────────────────────────────────────── */
typedef enum {
    PENDING_CALL_NONE           = 0,
    PENDING_CALL_MOTOR_CAL      = CALL_MOTOR_CALIBRATION,
    PENDING_CALL_ENCODER_CAL    = CALL_ENCODER_CALIBRATION,
    PENDING_CALL_ENCODER_INDEX  = CALL_ENCODER_INDEX_SEARCH,
    PENDING_CALL_CLOSED_LOOP    = CALL_ENTER_CLOSED_LOOP,
    PENDING_CALL_CLEAR_ERRORS   = CALL_CLEAR_ERRORS,
    PENDING_CALL_SET_IDLE       = CALL_SET_IDLE,
    PENDING_CALL_APPLY_CONFIG   = CALL_APPLY_CONFIG,
    PENDING_CALL_SET_HOME       = CALL_SET_HOME,
    PENDING_CALL_FFB_TEST_ON    = CALL_FFB_TEST_ON,
    PENDING_CALL_FFB_TEST_OFF   = CALL_FFB_TEST_OFF,
} PendingCall_t;

extern volatile PendingCall_t g_pending_call;
extern volatile bool          g_pending_save;

/* Deferred GET_CONFIG response — set by ISR, sent by protocol_process_pending() */
extern volatile bool     g_pending_config_resp;
extern volatile uint16_t g_pending_config_pid;

/* ── Application callback table ─────────────────────────────────────────────── */
typedef struct {
    void (*set_axis_state)(uint8_t state);
    void (*set_input_pos) (float pos);
    void (*clear_errors)  (void);
    void (*enter_dfu)     (void);
    void (*apply_config)  (void);  /* copy g_config → ODrive live objects */
    void (*set_home)      (void);  /* capture current position as zero reference */
    void (*save_config)   (void);  /* persist config to flash (handles post-erase errors) */
    void (*ffb_test_on)   (void);  /* enable FFB actuators for bench testing */
    void (*ffb_test_off)  (void);  /* disable FFB actuators / end test mode  */
} ProtocolCallbacks_t;

/* ── Public API ──────────────────────────────────────────────────────────────── */
void protocol_init(const ProtocolCallbacks_t *callbacks);
void protocol_dispatch(const HID_Command_t *cmd);
void protocol_process_pending(void);
bool protocol_send_config_response(uint16_t param_id, float value);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
