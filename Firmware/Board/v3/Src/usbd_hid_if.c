#include "usbd_hid_if.h"
#include "ffb_pid.h"
#include <string.h>
#include "hid_queue.h"


/* ── Combined HID Report Descriptor ──────────────────────────────────────────
 *
 * Application Collection 1: Generic Desktop / Joystick  (Joystick + FFB PID)
 *   Report 0x01 (IN,  2 B)  — Joystick X axis
 *   Report 0x05 (Feat,10 B) — PID Set Effect
 *   Report 0x07 (Feat,12 B) — PID Set Condition (Spring / Damper)
 *   Report 0x08 (Feat, 3 B) — PID Set Constant Force
 *   Report 0x0B (Feat, 4 B) — PID Effect Operation
 *   Report 0x0C (Feat, 1 B) — PID Device Control
 *   Report 0x0D (Feat, 1 B) — PID Device Gain
 *   Report 0x0E (IN,   1 B) — PID State
 *   Report 0x0F (Feat, 4 B) — PID Block Load   (GET_REPORT, device→host)
 *   Report 0x10 (Feat, 4 B) — PID Pool Report  (GET_REPORT, device→host)
 *
 * Application Collection 2: Vendor 0xFF00  (unchanged)
 *   Report 0x02 (IN, 48 B) — Telemetry
 *   Report 0x03 (Feat,8 B) — Command (host→device)
 *   Report 0x04 (IN,  6 B) — Config Response
 *
 * Total descriptor size must equal HID_REPORT_DESC_SIZE in usbd_hid.h.
 * ─────────────────────────────────────────────────────────────────────────── */
/* NOTE: HID_REPORT_DESC_SIZE must equal the number of initialiser bytes below.
 * If you add/remove bytes and the count is LARGER than HID_REPORT_DESC_SIZE the
 * compiler will error ("excess elements in array initializer").
 * If smaller, the tail is zero-filled and the USB host may reject the descriptor.
 * Verified count: 470 bytes (417 Joystick+PID + 53 Vendor). */
__ALIGN_BEGIN uint8_t HID_JOY_ReportDesc[] __ALIGN_END = {

/* Joystick */
0x05, 0x01,
0x09, 0x04,
0xA1, 0x01,

  0x85, 0x01,
  0x09, 0x30,
  0x16, 0x00, 0x80,
  0x26, 0xFF, 0x7F,
  0x75, 0x10,
  0x95, 0x01,
  0x81, 0x02,

0xC0
};
__ALIGN_BEGIN uint8_t HID_FFB_ReportDesc[] __ALIGN_END = {

0x05, 0x0F,
0x09, 0x92,
0xA1, 0x01,

  /* Set Effect */
  0x85, 0x05,
  0x09, 0x22,
  0x15, 0x00,
  0x25, 0x04,
  0x75, 0x08,
  0x95, 0x01,
  0xB1, 0x02,

  /* Effect Type */
  0x09, 0x25,
  0xA1, 0x02,
    0x09, 0x26,
    0x09, 0x40,
    0x09, 0x41,
    0x15, 0x01,
    0x25, 0x03,
    0x75, 0x08,
    0x95, 0x01,
    0xB1, 0x00,
  0xC0,

  /* Constant Force */
  0x85, 0x08,
  0x09, 0x70,
  0x16, 0xF0, 0xD8,
  0x26, 0x10, 0x27,
  0x75, 0x10,
  0x95, 0x01,
  0xB1, 0x02,

  /* Device Control */
  0x85, 0x0C,
  0x09, 0x96,
  0xA1, 0x02,
    0x09, 0x97,
    0x09, 0x98,
    0x09, 0x99,
    0x09, 0x9A,
    0x09, 0x9B,
    0x09, 0x9C,
    0x15, 0x01,
    0x25, 0x06,
    0x75, 0x08,
    0x95, 0x01,
    0xB1, 0x00,
  0xC0,

  /* PID State */
  0x85, 0x0E,
  0x09, 0x90,
  0xA1, 0x02,
    0x09, 0x92,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x01,
    0x81, 0x02,
    0x75, 0x07,
    0x95, 0x01,
    0x81, 0x03,
  0xC0,

0xC0
};
__ALIGN_BEGIN uint8_t HID_VENDOR_ReportDesc[] __ALIGN_END = {

0x06, 0x00, 0xFF,
0x09, 0x01,
0xA1, 0x01,

  /* Telemetry */
  0x85, 0x02,
  0x09, 0x02,
  0x75, 0x08,
  0x95, 0x30,
  0x81, 0x02,

  /* Command */
  0x85, 0x03,
  0x09, 0x03,
  0x75, 0x08,
  0x95, 0x08,
  0xB1, 0x02,

0xC0
};

/* ── HID_Joystick_Send ───────────────────────────────────────────────────── */
uint8_t HID_Joystick_Send(int16_t value)
{
    uint8_t report[3];

    report[0] = HID_REPORT_ID_JOYSTICK;
    report[1] = (uint8_t)(value & 0xFF);
    report[2] = (uint8_t)(value >> 8);

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
    return USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}

/* ── HID_ODrive_ProcessCommand (weak default) ────────────────────────────── */
__attribute__((weak)) void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *cmd)
{
    (void)cmd;
}
