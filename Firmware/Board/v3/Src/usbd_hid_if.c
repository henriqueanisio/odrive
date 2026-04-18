#include "usbd_hid_if.h"
#include "ffb_pid.h"
#include <string.h>
#include "hid_queue.h"

/* ── Combined HID Report Descriptor ──────────────────────────────────────────
 *
 * TWO top-level Application Collections (TLCs) — one HID interface.
 *
 * TLC1 is GENERIC_DESKTOP/JOYSTICK (UP:0001/U:0004).  DirectInput requires
 * the FFB Output reports to appear inside a JOYSTICK Application Collection —
 * not inside a PID (UP:000F) Application Collection — to set DIDC_FORCEFEEDBACK.
 * This matches the stm32_ffb_wheel reference implementation that is confirmed
 * working.
 *
 *   TLC1 — Generic Desktop / Joystick  (UP:0001 U:0004)  1244 bytes
 *       Report 0x40 IN  (3 B)  8 buttons + X (steering) axis
 *       Report 0x02 IN  (1 B)  PID State
 *       Reports 0x01-0x0D OUT  FFB effect/control
 *       Report  0x07 FEATURE   Create New Effect (SET_REPORT)
 *       Report  0x08 FEATURE   PID Block Load    (GET_REPORT)
 *       Report  0x09 FEATURE   PID Pool          (GET_REPORT)
 *
 *   TLC2 — Vendor 0xFF00               (UP:FF00 U:0001)   53 bytes
 *       Report 0x20 IN  (52 B) Telemetry
 *       Report 0x22 IN  ( 6 B) Config Response
 *       Report 0x21 FEATURE (8 B) Command
 *
 * Total = 1244 + 53 = 1297 bytes = HID_REPORT_DESC_SIZE.
 * ─────────────────────────────────────────────────────────────────────────── */
const uint8_t HID_ReportDesc[] = {

  // ===== JOYSTICK TLC =====
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x04,        // Usage (Joystick)
  0xA1, 0x01,        // Collection (Application)

  // ===== INPUT (Joystick) =====
  0x85, 0x01,        // Report ID 1

  // Buttons (8)
  0x05, 0x09,
  0x19, 0x01,
  0x29, 0x08,
  0x15, 0x00,
  0x25, 0x01,
  0x75, 0x01,
  0x95, 0x08,
  0x81, 0x02,

  // Padding
  0x75, 0x08,
  0x95, 0x01,
  0x81, 0x03,

  // Axis X (16 bits)
  0x05, 0x01,
  0x09, 0x30,
  0x16, 0x00, 0x80,
  0x26, 0xFF, 0x7F,
  0x75, 0x10,
  0x95, 0x01,
  0x81, 0x02,

  // ===== PID STATE (Input) =====
  0x05, 0x0F,
  0x09, 0x92,
  0xA1, 0x02,
    0x85, 0x02,
    0x09, 0x9F,
    0x09, 0xA0,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x02,
    0x81, 0x02,

    // padding
    0x75, 0x06,
    0x95, 0x01,
    0x81, 0x03,
  0xC0,

  // ===== SET EFFECT =====
  0x05, 0x0F,
  0x09, 0x21,
  0xA1, 0x02,
    0x85, 0x03,

    // Effect Block Index
    0x09, 0x22,
    0x15, 0x01,
    0x25, 0x28,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,

    // Effect Type
    0x09, 0x25,
    0xA1, 0x02,
      0x09, 0x26, // Constant Force
      0x09, 0x27, // Ramp
      0x15, 0x01,
      0x25, 0x02,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x00,
    0xC0,

    // Duration
    0x09, 0x50,
    0x16, 0x00, 0x00,
    0x26, 0xFF, 0x7F,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

    // Gain
    0x09, 0x52,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,

  0xC0,

  // ===== CONSTANT FORCE =====
  0x05, 0x0F,
  0x09, 0x73,
  0xA1, 0x02,
    0x85, 0x04,

    // Effect Block Index
    0x09, 0x22,
    0x15, 0x01,
    0x25, 0x28,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,

    // Magnitude
    0x09, 0x70,
    0x16, 0xF0, 0xD8,
    0x26, 0x10, 0x27,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

  0xC0,

  // ===== DEVICE CONTROL =====
  0x05, 0x0F,
  0x09, 0x96,
  0xA1, 0x02,
    0x85, 0x05,
    0x09, 0x97,
    0x09, 0x98,
    0x09, 0x9A,
    0x15, 0x01,
    0x25, 0x03,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x00,
  0xC0,

  // ===== DEVICE GAIN =====
  0x05, 0x0F,
  0x09, 0x7D,
  0xA1, 0x02,
    0x85, 0x06,
    0x09, 0x7E,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,
  0xC0,

  0xC0  // END JOYSTICK
};

/* ── HID_Joystick_Send ───────────────────────────────────────────────────── */
uint8_t HID_Joystick_Send(int16_t value)
{
    uint8_t report[1 + HID_JOYSTICK_PAYLOAD_SIZE];
    report[0] = HID_REPORT_ID_JOYSTICK;
    report[1] = 0;                          /* buttons byte — all released    */
    report[2] = (uint8_t)(value & 0xFF);    /* X axis low byte                */
    report[3] = (uint8_t)(value >> 8);      /* X axis high byte               */
    return hid_queue_push(report, sizeof(report));
}

/* ── HID_ODrive_SendTelemetry ────────────────────────────────────────────── */
uint8_t HID_ODrive_SendTelemetry(const HID_TelemetryPayload_t *payload)
{
    uint8_t report[1 + HID_TELEMETRY_PAYLOAD_SIZE];
    report[0] = HID_REPORT_ID_TELEMETRY;
    memcpy(&report[1], payload, HID_TELEMETRY_PAYLOAD_SIZE);
    return hid_queue_push(report, sizeof(report));
}

/* ── HID_ODrive_SendConfigResponse ──────────────────────────────────────── */
uint8_t HID_ODrive_SendConfigResponse(uint16_t param_id, float value)
{
    uint8_t report[1 + 2 + 4];
    report[0] = HID_REPORT_ID_CONFIG_RESP;
    memcpy(&report[1], &param_id, 2);
    memcpy(&report[3], &value,    4);
    bool ok = hid_queue_push(report, sizeof(report));
    hid_queue_process();
    return ok ? (uint8_t)USBD_OK : (uint8_t)USBD_BUSY;
}

/* ── HID_ODrive_ProcessCommand (weak default) ────────────────────────────── */
__attribute__((weak)) void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *cmd)
{
    (void)cmd;
}
