#ifndef __USBD_HID_IF_H
#define __USBD_HID_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_hid.h"
#include "usb_device.h"
#include <stdint.h>

/* ── Report IDs ────────────────────────────────────────────────────────────── */
#define HID_REPORT_ID_JOYSTICK    0x01U   /* Input:   joystick axis (2 bytes)  */
#define HID_REPORT_ID_TELEMETRY   0x02U   /* Input:   device → host (37 bytes) */
#define HID_REPORT_ID_COMMAND     0x03U   /* Feature: host → device  (6 bytes) */

/* ── Command IDs (received in Feature Report byte[0]) ─────────────────────── */
#define HID_CMD_SET_AXIS_STATE    0x01U
#define HID_CMD_CLEAR_ERRORS      0x02U
#define HID_CMD_SAVE_CONFIG       0x03U
#define HID_CMD_REBOOT            0x04U
#define HID_CMD_ENTER_DFU         0x05U
#define HID_CMD_SET_INPUT_POS     0x06U
#define HID_CMD_SET_CURRENT_LIM   0x07U

/* ── Payload sizes ─────────────────────────────────────────────────────────── */
#define HID_TELEMETRY_PAYLOAD_SIZE  37U   /* bytes after report_id */
#define HID_COMMAND_PAYLOAD_SIZE     7U   /* bytes after report_id */

/*
 * Telemetry input report payload layout (37 bytes, little-endian, packed):
 *   float  pos_estimate
 *   float  vel_estimate
 *   float  vbus_voltage
 *   float  current_lim
 *   float  input_pos
 *   float  Iq_measured
 *   float  phase_resistance
 *   float  phase_inductance
 *   uint8  current_state
 *   uint8  flags            (bit0=enc_ready  bit1=motor_calibrated)
 *   uint8  axis_error       (0 = no error)
 *   uint8  motor_error      (0 = no error)
 *   uint8  encoder_error    (0 = no error)
 *
 * Command feature report payload layout (6 bytes, little-endian, packed):
 *   uint8  cmd_id
 *   float  param
 *   uint8  reserved
 */

typedef struct __attribute__((packed)) {
    float   pos_estimate;
    float   vel_estimate;
    float   vbus_voltage;
    float   current_lim;
    float   input_pos;
    float   Iq_measured;
    float   phase_resistance;
    float   phase_inductance;
    uint8_t current_state;
    uint8_t flags;
    uint8_t axis_error;
    uint8_t motor_error;
    uint8_t encoder_error;
} HID_TelemetryPayload_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd_id;
    float   param;
    uint8_t reserved;
} HID_CommandPayload_t;

/*
 * HID_Joystick_Send()
 *   Send the 16-bit signed joystick X axis (Report ID 0x01).
 *   Compatible with joy.cpl / Windows game controller.
 */
uint8_t HID_Joystick_Send(int16_t value);

/*
 * HID_ODrive_SendTelemetry()
 *   Call periodically (e.g. every 10 ms) to push telemetry to the host.
 */
uint8_t HID_ODrive_SendTelemetry(const HID_TelemetryPayload_t *payload);

/*
 * HID_ODrive_ProcessCommand()
 *   Called from USB interrupt when a Feature Report is received.
 *   Provide a strong definition in your application (e.g. communication.cpp).
 *   A default __weak no-op is defined in usbd_hid_if.c.
 */
void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *cmd);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_HID_IF_H */
