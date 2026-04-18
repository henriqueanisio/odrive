#ifndef __USBD_HID_IF_H
#define __USBD_HID_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_hid.h"
#include "usb_device.h"
#include <stdint.h>

/* ── Report IDs ──────────────────────────────────────────────────────────────
 *   0x01 — Joystick  Input  (2 bytes)    Generic Desktop — joy.cpl
 *   0x01..0x0D — FFB OUT/IN             PID — joy.cpl FFB tab
 *   0x11..0x13 — FFB Feature            PID — block load/pool
 *   0x20 — Telemetry Input (52 bytes)   Vendor 0xFF00   — GUI live data
 *   0x21 — Command   Feature (8 bytes)  Vendor 0xFF00   — host → device
 *   0x22 — CfgResp   Input   (6 bytes)  Vendor 0xFF00   — GET_CONFIG reply
 * ─────────────────────────────────────────────────────────────────────────── */
#define HID_REPORT_ID_JOYSTICK    0x40U  /* 0x40 avoids global Report ID conflict with PID 0x01 */
#define HID_REPORT_ID_TELEMETRY   0x20U  /* renamed: 0x02-0x13 reserved for FFB */
#define HID_REPORT_ID_COMMAND     0x21U
#define HID_REPORT_ID_CONFIG_RESP 0x22U

/* ── Payload sizes (bytes after the report_id byte) ─────────────────────── */
#define HID_JOYSTICK_PAYLOAD_SIZE   3U
#define HID_TELEMETRY_PAYLOAD_SIZE 52U
#define HID_COMMAND_PAYLOAD_SIZE    8U
#define HID_CONFIG_RESP_PAYLOAD_SIZE 6U

/* ── Telemetry input payload (46 bytes, little-endian packed) ────────────────
 *
 *   offset  size  field
 *    0       4    float   pos_estimate
 *    4       4    float   vel_estimate
 *    8       4    float   vbus_voltage
 *   12       4    float   current_lim
 *   16       4    float   input_pos
 *   20       4    float   Iq_measured
 *   24       4    float   phase_resistance
 *   28       4    float   phase_inductance
 *   32       1    uint8   current_state
 *   33       1    uint8   flags         bit0=enc_ready  bit1=motor_calibrated
 *   34       4    uint32  axis_error    Axis::Error bitmask (0=no error)
 *   38       4    uint32  motor_error   Motor::Error bitmask (0=no error)
 *   42       4    uint32  encoder_error Encoder::Error bitmask (0=no error)
 *   46       1    uint8   mag_agc       AS5047P AGC: 0=strong(close), 255=weak(far)
 *   47       1    uint8   mag_flags     bit0=COMP_H(too close), bit1=COMP_L(too far)
 *   48       2    uint16  ffb_rx_count  count of FFB Output reports received via DataOut
 *   50       1    uint8   ffb_last_rid  report ID of last received FFB Output report
 *   51       1    uint8   ffb_actv      bit0=actuators_enabled, bit1=any_effect_active
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    float    pos_estimate;
    float    vel_estimate;
    float    vbus_voltage;
    float    current_lim;
    float    input_pos;
    float    Iq_measured;
    float    phase_resistance;
    float    phase_inductance;
    uint8_t  current_state;
    uint8_t  flags;
    uint32_t axis_error;
    uint32_t motor_error;
    uint32_t encoder_error;
    uint8_t  mag_agc;       /* AS5047P: 0=strong field, 255=weak field, ~128=optimal */
    uint8_t  mag_flags;     /* bit0=COMP_H (too close), bit1=COMP_L (too far)        */
    uint16_t ffb_rx_count;  /* increments each time an FFB Output report arrives     */
    uint8_t  ffb_last_rid;  /* report ID of the last received FFB Output report      */
    uint8_t  ffb_actv;      /* bit0=actuators_enabled, bit1=any_effect_active        */
} HID_TelemetryPayload_t;          /* 52 bytes */

/* ── Command feature payload (8 bytes) ───────────────────────────────────────
 *   See protocol.h for cmd_id and param_id definitions.
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  cmd_id;
    uint16_t param_id;
    float    value;
    uint8_t  reserved;
} HID_CommandPayload_t;             /* 8 bytes */

/* ── Config response payload (6 bytes) ───────────────────────────────────────
 *   Full report sent as [0x22 | uint16 param_id | float value].
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint16_t param_id;
    float    value;
} HID_ConfigResponsePayload_t;           /* 7 bytes including report_id */

/* ── Public API ─────────────────────────────────────────────────────────────
 *
 * HID_Joystick_Send()
 *   Send int16 axis (Report 0x01) — keeps joy.cpl working.
 *
 * HID_ODrive_SendTelemetry()
 *   Send full telemetry (Report 0x20) — call every ~10 ms from task.
 *
 * HID_ODrive_SendConfigResponse()
 *   Send a GET_CONFIG reply (Report 0x22) — called from protocol_dispatch.
 *
 * HID_ODrive_ProcessCommand()
 *   Weak default (no-op) defined in usbd_hid_if.c.
 *   Override in hid_app_integration.cpp to handle commands.
 * ─────────────────────────────────────────────────────────────────────────── */
uint8_t HID_Joystick_Send           (int16_t value);
uint8_t HID_ODrive_SendTelemetry    (const HID_TelemetryPayload_t *payload);
uint8_t HID_ODrive_SendConfigResponse(uint16_t param_id, float value);
void    HID_ODrive_ProcessCommand   (const HID_CommandPayload_t *cmd);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_HID_IF_H */