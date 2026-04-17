#include "usbd_hid_if.h"
#include "ffb_pid.h"
#include <string.h>
#include "hid_queue.h"

/* ── Combined HID Report Descriptor ──────────────────────────────────────────
 *
 * Structure confirmed by OpenFFBoard reference implementation:
 *   - All FFB/PID reports in the Joystick Application Collection (TLC1)
 *   - Each FFB report wrapped in a Logical Collection with the PID report
 *     usage (0x21, 0x6E, 0x5F, 0x73, 0x77, 0x95, 0x92, 0x89, 0x7F, …)
 *     DirectInput scans for these usages to detect FFB capability.
 *   - Effect Block Index uses Logical Min 1 (index 0 is invalid per spec)
 *
 * Application Collection 1: Generic Desktop / Joystick + PID
 *   Report 0x01 (IN,  2 B)  — Joystick X axis
 *   Report 0x05 (Out, 10 B) — PID Set Effect
 *   Report 0x06 (Out,  9 B) — PID Set Periodic
 *   Report 0x07 (Out, 12 B) — PID Set Condition
 *   Report 0x08 (Out,  3 B) — PID Set Constant Force
 *   Report 0x0B (Out,  4 B) — PID Effect Operation
 *   Report 0x0C (Out,  1 B) — PID Device Control
 *   Report 0x0D (Out,  1 B) — PID Device Gain
 *   Report 0x0E (IN,   1 B) — PID State
 *   Report 0x0F (Feat, 4 B) — PID Block Load   (device→host)
 *   Report 0x10 (Feat, 4 B) — PID Pool Report  (device→host)
 *
 * Application Collection 2: Vendor 0xFF00  (53 bytes)
 *   Report 0x02 (IN, 52 B) — Telemetry
 *   Report 0x03 (Feat,8 B) — Command
 *   Report 0x04 (IN,  6 B) — Config Response
 *
 * Total = HID_REPORT_DESC_SIZE in usbd_hid.h.
 * ─────────────────────────────────────────────────────────────────────────── */
__ALIGN_BEGIN uint8_t HID_ReportDesc[HID_REPORT_DESC_SIZE] __ALIGN_END = {

    /* ═══════════════════════════════════════════════════════════════════════
     * Application Collection 1: Generic Desktop / Joystick + PID
     * ═══════════════════════════════════════════════════════════════════════ */
    0x05, 0x01,        /* Usage Page (Generic Desktop)                        */
    0x09, 0x04,        /* Usage (Joystick)                                    */
    0xA1, 0x01,        /* Collection (Application)                            */

      /* ── Report 0x01: Joystick X axis (IN, 2 bytes payload) ── */
      0x85, HID_REPORT_ID_JOYSTICK,
      0x09, 0x30,      /* Usage (X)                                           */
      0x16, 0x00, 0x80,/* Logical Minimum (-32768)                            */
      0x26, 0xFF, 0x7F,/* Logical Maximum (+32767)                            */
      0x75, 0x10,      /* Report Size (16)                                    */
      0x95, 0x01,      /* Report Count (1)                                    */
      0x81, 0x02,      /* Input (Variable, Absolute)                          */

      /* ── Switch to PID Usage Page for FFB reports ── */
      0x05, 0x0F,      /* Usage Page (Physical Interface Device)              */

      /* ── Report 0x05: Set Effect (Usage 0x21, Output, 10 bytes payload) ── */
      0x09, 0x21,      /* Usage (Set Effect Report)   ← DirectInput scans for this */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_SET_EFFECT,  /* Report ID                            */
        0x09, 0x22,    /* Usage (Effect Block Index)                          */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, FFB_MAX_EFFECTS,        /* Logical Maximum                      */
        0x35, 0x01,    /* Physical Minimum (1)                                */
        0x45, FFB_MAX_EFFECTS,        /* Physical Maximum                     */
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x25,    /* Usage (Effect Type)                                 */
        0xA1, 0x02,    /* Collection (Logical)                                */
          0x09, 0x26,  /* Usage (ET Constant Force) → selector 1             */
          0x09, 0x28,  /* Usage (ET Sine)           → selector 2             */
          0x09, 0x29,  /* Usage (ET Square)         → selector 3             */
          0x09, 0x2A,  /* Usage (ET Triangle)       → selector 4             */
          0x09, 0x40,  /* Usage (ET Spring)         → selector 5             */
          0x09, 0x41,  /* Usage (ET Damper)         → selector 6             */
          0x15, 0x01,  /* Logical Minimum (1)                                 */
          0x25, 0x06,  /* Logical Maximum (6)                                 */
          0x75, 0x08,  /* Report Size (8)                                     */
          0x95, 0x01,  /* Report Count (1)                                    */
          0x91, 0x00,  /* Output (Array)                                      */
        0xC0,          /* End Collection (Effect Type)                        */
        0x09, 0x50,    /* Usage (Duration)                                    */
        0x09, 0x54,    /* Usage (Trigger Repeat Interval)                     */
        0x09, 0x51,    /* Usage (Sample Period)                               */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x27, 0xFF, 0xFF, 0x00, 0x00, /* Logical Maximum (65535)             */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x03,    /* Report Count (3)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x52,    /* Usage (Gain)                                        */
        0x09, 0x53,    /* Usage (Trigger Button)                              */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x26, 0xFF, 0x00, /* Logical Maximum (255)                           */
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x02,    /* Report Count (2)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
      0xC0,            /* End Collection (Set Effect)                         */

      /* ── Report 0x06: Set Periodic (Usage 0x6E, Output, 9 bytes) ── */
      0x09, 0x6E,      /* Usage (Set Periodic Report)                         */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_SET_PERIODIC,
        0x09, 0x22,    /* Usage (Effect Block Index)                          */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,    /* Physical Minimum (1)                                */
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x70,    /* Usage (Magnitude)                                   */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x26, 0x10, 0x27, /* Logical Maximum (10000)                         */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x60,    /* Usage (Offset)                                      */
        0x16, 0xF0, 0xD8, /* Logical Minimum (-10000)                        */
        0x26, 0x10, 0x27, /* Logical Maximum (+10000)                        */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x68,    /* Usage (Phase)                                       */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x27, 0x9F, 0x8C, 0x00, 0x00, /* Logical Maximum (35999)            */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x50,    /* Usage (Period)                                      */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x27, 0xFF, 0xFF, 0x00, 0x00, /* Logical Maximum (65535)            */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
      0xC0,            /* End Collection (Set Periodic)                       */

      /* ── Report 0x07: Set Condition (Usage 0x5F, Output, 12 bytes) ── */
      0x09, 0x5F,      /* Usage (Set Condition Report)                        */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_SET_CONDITION,
        0x09, 0x22,    /* Usage (Effect Block Index)                          */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,    /* Physical Minimum (1)                                */
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x23,    /* Usage (Parameter Block Offset)                      */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x25, 0x01,    /* Logical Maximum (1)                                 */
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x60,    /* Usage (CP Offset)                                   */
        0x16, 0xF0, 0xD8, /* Logical Minimum (-10000)                        */
        0x26, 0x10, 0x27, /* Logical Maximum (+10000)                        */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x61,    /* Usage (Positive Coefficient)                        */
        0x09, 0x62,    /* Usage (Negative Coefficient)                        */
        0x16, 0xF0, 0xD8,
        0x26, 0x10, 0x27,
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x02,    /* Report Count (2)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x63,    /* Usage (Positive Saturation)                         */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x26, 0x10, 0x27,
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x65,    /* Usage (Dead Band)                                   */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x26, 0x10, 0x27,
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
      0xC0,            /* End Collection (Set Condition)                      */

      /* ── Report 0x08: Set Constant Force (Usage 0x73, Output, 3 bytes) ── */
      0x09, 0x73,      /* Usage (Set Constant Force Report)                   */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_SET_CONSTANT_FORCE,
        0x09, 0x22,    /* Usage (Effect Block Index)                          */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,    /* Physical Minimum (1)                                */
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x70,    /* Usage (Magnitude)                                   */
        0x16, 0xF0, 0xD8, /* Logical Minimum (-10000)                        */
        0x26, 0x10, 0x27, /* Logical Maximum (+10000)                        */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
      0xC0,            /* End Collection (Set Constant Force)                 */

      /* ── Report 0x0B: Effect Operation (Usage 0x77, Output, 4 bytes) ── */
      0x09, 0x77,      /* Usage (Effect Operation Report)                     */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_EFFECT_OPERATION,
        0x09, 0x22,    /* Usage (Effect Block Index)                          */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,    /* Physical Minimum (1)                                */
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x78,    /* Usage (Op Effect Start)   — Named Array             */
        0xA1, 0x02,    /* Collection (Logical)                                */
          0x09, 0x79,  /* Usage (Op Effect Start)       → selector 1         */
          0x09, 0x7A,  /* Usage (Op Effect Start Solo)  → selector 2         */
          0x09, 0x7B,  /* Usage (Op Effect Stop)        → selector 3         */
          0x15, 0x01,  /* Logical Minimum (1)                                 */
          0x25, 0x03,  /* Logical Maximum (3)                                 */
          0x75, 0x08,  /* Report Size (8)                                     */
          0x95, 0x01,  /* Report Count (1)                                    */
          0x91, 0x00,  /* Output (Array)                                      */
        0xC0,          /* End Collection                                      */
        0x09, 0x7C,    /* Usage (Loop Count)                                  */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x27, 0xFF, 0xFF, 0x00, 0x00, /* Logical Maximum (65535)             */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
      0xC0,            /* End Collection (Effect Operation)                   */

      /* ── Report 0x0C: Device Control (Usage 0x95, Output, 1 byte) ── */
      0x09, 0x95,      /* Usage (PID Device Control)                          */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_DEVICE_CONTROL,
        0x09, 0x96,    /* Usage (PID Device Control)  — sub-collection        */
        0xA1, 0x02,    /* Collection (Logical)                                */
          0x09, 0x97,  /* Usage (DC Enable Actuators)   → selector 1         */
          0x09, 0x98,  /* Usage (DC Disable Actuators)  → selector 2         */
          0x09, 0x99,  /* Usage (DC Stop All Effects)   → selector 3         */
          0x09, 0x9A,  /* Usage (DC Device Reset)       → selector 4         */
          0x09, 0x9B,  /* Usage (DC Device Pause)       → selector 5         */
          0x09, 0x9C,  /* Usage (DC Continue)           → selector 6         */
          0x15, 0x01,  /* Logical Minimum (1)                                 */
          0x25, 0x06,  /* Logical Maximum (6)                                 */
          0x75, 0x08,  /* Report Size (8)                                     */
          0x95, 0x01,  /* Report Count (1)                                    */
          0x91, 0x00,  /* Output (Array)                                      */
        0xC0,          /* End Collection (DC sub)                             */
      0xC0,            /* End Collection (Device Control)                     */

      /* ── Report 0x0D: Device Gain (Usage 0x7D, Output, 1 byte) ── */
      0x09, 0x7D,      /* Usage (Device Gain Report)                          */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_DEVICE_GAIN,
        0x09, 0x7E,    /* Usage (Device Gain)                                 */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x26, 0xFF, 0x00, /* Logical Maximum (255)                           */
        0x35, 0x00,    /* Physical Minimum (0)                                */
        0x46, 0x10, 0x27, /* Physical Maximum (10000)                        */
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
      0xC0,            /* End Collection (Device Gain)                        */

      /* ── Report 0x0E: PID State (Usage 0x92, Input, 1 byte) ── */
      0x05, 0x0F,      /* Usage Page (Physical Interface Device)              */
      0x09, 0x92,      /* Usage (PID State Report)                            */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_PID_STATE,
        0x09, 0xA0,    /* Usage (Actuators Enabled)  → bit 0                 */
        0x09, 0x94,    /* Usage (Effect Playing)     → bit 1                 */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x25, 0x01,    /* Logical Maximum (1)                                 */
        0x75, 0x01,    /* Report Size (1)                                     */
        0x95, 0x02,    /* Report Count (2)                                    */
        0x81, 0x02,    /* Input (Variable)                                    */
        0x75, 0x06,    /* Report Size (6) — padding                           */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x81, 0x03,    /* Input (Constant)                                    */
      0xC0,            /* End Collection (PID State)                          */

      /* ── Report 0x0F: PID Block Load (Usage 0x89, Feature, device→host) ── */
      0x05, 0x0F,      /* Usage Page (Physical Interface Device)              */
      0x09, 0x89,      /* Usage (Block Load Report)                           */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_PID_BLOCK_LOAD,
        0x09, 0x22,    /* Usage (Effect Block Index)                          */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,    /* Physical Minimum (1)                                */
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0xB1, 0x02,    /* Feature (Variable)                                  */
        0x09, 0x8B,    /* Usage (Block Load Status)                           */
        0xA1, 0x02,    /* Collection (Logical)                                */
          0x09, 0x8C,  /* Usage (Block Load Success) → selector 1            */
          0x09, 0x8D,  /* Usage (Block Load Full)    → selector 2            */
          0x09, 0x8E,  /* Usage (Block Load Error)   → selector 3            */
          0x15, 0x01,  /* Logical Minimum (1)                                 */
          0x25, 0x03,  /* Logical Maximum (3)                                 */
          0x75, 0x08,  /* Report Size (8)                                     */
          0x95, 0x01,  /* Report Count (1)                                    */
          0xB1, 0x00,  /* Feature (Array)                                     */
        0xC0,          /* End Collection                                      */
        0x09, 0xAC,    /* Usage (RAM Pool Available)                          */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x27, 0xFF, 0xFF, 0x00, 0x00, /* Logical Maximum (65535)             */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0xB1, 0x02,    /* Feature (Variable)                                  */
      0xC0,            /* End Collection (Block Load)                         */

      /* ── Report 0x10: PID Pool (Usage 0x7F, Feature, device→host) ── */
      0x09, 0x7F,      /* Usage (PID Pool Report)                             */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, FFB_REPORT_PID_POOL,
        0x09, 0x80,    /* Usage (RAM Pool Size)                               */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x27, 0xFF, 0xFF, 0x00, 0x00, /* Logical Maximum (65535)             */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x01,    /* Report Count (1)                                    */
        0xB1, 0x02,    /* Feature (Variable)                                  */
        0x09, 0x83,    /* Usage (Simultaneous Effects Max)                    */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x26, 0xFF, 0x00, /* Logical Maximum (255)                           */
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0xB1, 0x02,    /* Feature (Variable)                                  */
        0x09, 0xA9,    /* Usage (Device Managed Pool)                         */
        0x09, 0xAA,    /* Usage (Shared Parameter Blocks)                     */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x25, 0x01,    /* Logical Maximum (1)                                 */
        0x75, 0x01,    /* Report Size (1)                                     */
        0x95, 0x02,    /* Report Count (2)                                    */
        0xB1, 0x02,    /* Feature (Variable)                                  */
        0x75, 0x06,    /* Report Size (6) — padding                           */
        0x95, 0x01,    /* Report Count (1)                                    */
        0xB1, 0x03,    /* Feature (Constant)                                  */
      0xC0,            /* End Collection (PID Pool)                           */

    0xC0,              /* End Collection (Joystick + PID Application)         */

    /* ═══════════════════════════════════════════════════════════════════════
     * Application Collection 2: Vendor 0xFF00
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

    0xC0,              /* End Collection (Vendor)                             */
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
    bool ok = hid_queue_push(report, sizeof(report));
    hid_queue_process();
    return ok ? (uint8_t)USBD_OK : (uint8_t)USBD_BUSY;
}

/* ── HID_ODrive_ProcessCommand (weak default) ────────────────────────────── */
__attribute__((weak)) void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *cmd)
{
    (void)cmd;
}
