#include "usbd_hid_if.h"
#include "ffb_pid.h"
#include <string.h>
#include "hid_queue.h"

/* ── Combined HID Report Descriptor ──────────────────────────────────────────
 *
 * THREE separate top-level Application Collections (TLCs) — one HID interface.
 *
 * Confirmed by hardware ID analysis: HID_DEVICE_SYSTEM_PID is ONLY generated
 * when a top-level Application Collection with Usage Page 0x0F exists.
 * The OpenFFBoard single-TLC approach does NOT generate HID_DEVICE_SYSTEM_PID
 * and therefore does NOT produce the joy.cpl FF tab.
 *
 *   TLC1 — Generic Desktop / Joystick  (UP:0001 U:0004)  44 bytes
 *       Generates HID_DEVICE_SYSTEM_GAME → hidgame.sys → joy.cpl game tab
 *       Report 0x40 IN  (3 B)  8 buttons + X axis
 *       (0x40 avoids global Report ID conflict with PID's 0x01)
 *
 *   TLC2 — Physical Interface Device   (UP:000F U:0001)  1027 bytes
 *       Generates HID_DEVICE_SYSTEM_PID → hidpid.sys → joy.cpl FF tab ← KEY
 *       Report 0x02 IN  (1 B)  PID State
 *       Reports 0x01-0x0D OUT  FFB effect/control
 *       Reports 0x11-0x13 FEATURE  Create/Block/Pool
 *
 *   TLC3 — Vendor 0xFF00               (UP:FF00 U:0001)  53 bytes
 *       Report 0x20 IN  (52 B) Telemetry
 *       Report 0x22 IN  ( 6 B) Config Response
 *       Report 0x21 FEATURE (8 B) Command
 *
 * Total = 44 + 1027 + 53 = 1124 bytes = HID_REPORT_DESC_SIZE.
 * ─────────────────────────────────────────────────────────────────────────── */
__ALIGN_BEGIN uint8_t HID_ReportDesc[HID_REPORT_DESC_SIZE] __ALIGN_END = {

    /* ═══════════════════════════════════════════════════════════════════════
     * TLC1 — Generic Desktop / Joystick  (44 bytes)
     * Report 0x40: 8 buttons (1 B) + X axis (2 B) = 3 B payload
     * Uses 0x40 to avoid global Report ID conflict with PID's 0x01.
     * ═══════════════════════════════════════════════════════════════════════ */
    0x05, 0x01,        /* Usage Page (Generic Desktop)                        */
    0x09, 0x04,        /* Usage (Joystick)                                    */
    0xA1, 0x01,        /* Collection (Application)                            */
      0xA1, 0x00,      /* Collection (Physical)                               */
        0x85, 0x40,    /* Report ID (0x40)                                    */
        0x05, 0x09,    /* Usage Page (Button)                                 */
        0x19, 0x01,    /* Usage Minimum (1)                                   */
        0x29, 0x08,    /* Usage Maximum (8)                                   */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x25, 0x01,    /* Logical Maximum (1)                                 */
        0x75, 0x01,    /* Report Size (1)                                     */
        0x95, 0x08,    /* Report Count (8) — 8 buttons = 1 byte              */
        0x81, 0x02,    /* Input (Variable)                                    */
        0x05, 0x01,    /* Usage Page (Generic Desktop)                        */
        0x09, 0x30,    /* Usage (X)                                           */
        0x16, 0x00, 0x80, /* Logical Minimum (-32768)                         */
        0x26, 0xFF, 0x7F, /* Logical Maximum (32767)                          */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x81, 0x02,    /* Input (Variable, Absolute)                          */
      0xC0,            /* End Collection (Physical)                           */
    0xC0,              /* End Collection (TLC1 Joystick)                      */

    /* ================================
    * TLC2 — PID (FFB MINIMAL WORKING)
    * ================================ */
    0x05, 0x0F,        // Usage Page (Physical Interface)
    0x09, 0x01,        // Usage (Physical Interface Device)
    0xA1, 0x01,        // Collection (Application)

    /* --- PID State (INPUT) --- */
    0x85, 0x02,        // Report ID 2
    0x09, 0x92,        // PID State Report
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x08,
    0x81, 0x02,

    /* --- Set Effect (OUTPUT) --- */
    0x85, 0x01,
    0x09, 0x21,
    0xA1, 0x02,
    0x09, 0x22,        // Effect Block Index
    0x15, 0x01,
    0x25, 0x10,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,

    0x09, 0x25,        // Effect Type
    0x15, 0x01,
    0x25, 0x01,        // ONLY Constant Force
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,

    0xC0,

    /* --- Set Constant Force --- */
    0x85, 0x05,
    0x09, 0x73,
    0xA1, 0x02,
    0x09, 0x22,
    0x15, 0x01,
    0x25, 0x10,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,

    0x09, 0x70,        // Magnitude
    0x16, 0x01, 0x80,
    0x26, 0xFF, 0x7F,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,
    0xC0,

    /* --- Effect Operation --- */
    0x85, 0x0A,
    0x09, 0x77,
    0xA1, 0x02,
    0x09, 0x22,
    0x15, 0x01,
    0x25, 0x10,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,

    0x09, 0x78,
    0x15, 0x01,
    0x25, 0x03,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,
    0xC0,

    /* --- PID Pool (FEATURE) ESSENCIAL --- */
    0x85, 0x11,
    0x09, 0x7F,
    0xA1, 0x02,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x08,
    0xB1, 0x02,
    0xC0,

    /* --- Create New Effect (FEATURE) --- */
    0x85, 0x12,
    0x09, 0xAB,
    0xA1, 0x02,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x04,
    0xB1, 0x02,
    0xC0,

    0xC0,

    /* ═══════════════════════════════════════════════════════════════════════
     * TLC3 — Vendor 0xFF00  (UP:FF00 U:0001)  53 bytes
     * ═══════════════════════════════════════════════════════════════════════ */
    0x06, 0x00, 0xFF,  /* Usage Page (Vendor Defined 0xFF00)                  */
    0x09, 0x01,        /* Usage (Vendor 1)                                    */
    0xA1, 0x01,        /* Collection (Application)                            */

      0x85, HID_REPORT_ID_TELEMETRY,
      0x09, 0x02,
      0x15, 0x00,
      0x26, 0xFF, 0x00,
      0x75, 0x08,
      0x95, HID_TELEMETRY_PAYLOAD_SIZE,
      0x81, 0x02,

      0x85, HID_REPORT_ID_CONFIG_RESP,
      0x09, 0x04,
      0x15, 0x00,
      0x26, 0xFF, 0x00,
      0x75, 0x08,
      0x95, HID_CONFIG_RESP_PAYLOAD_SIZE,
      0x81, 0x02,

      0x85, HID_REPORT_ID_COMMAND,
      0x09, 0x03,
      0x15, 0x00,
      0x26, 0xFF, 0x00,
      0x75, 0x08,
      0x95, HID_COMMAND_PAYLOAD_SIZE,
      0xB1, 0x02,

    0xC0,              /* End Collection (TLC3 Vendor)                        */
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
