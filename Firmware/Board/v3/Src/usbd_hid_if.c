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

    /* ═══════════════════════════════════════════════════════════════════════
     * TLC2 — Physical Interface Device  (UP:000F U:0001)  1027 bytes
     * Separate top-level Application → Windows generates HID_DEVICE_SYSTEM_PID
     * → hidpid.sys loads → joy.cpl FF tab appears.
     * ═══════════════════════════════════════════════════════════════════════ */
    0x05, 0x0F,        /* Usage Page (Physical Interface Device)              */
    0x09, 0x01,        /* Usage (Physical Interface Device)                   */
    0xA1, 0x01,        /* Collection (Application)                            */

      /* ── PID State Report 0x02 INPUT (STATEREP, 37 bytes) ── */
      0x05, 0x0F,      /* Usage Page (Physical Interface Device)              */
      0x09, 0x92,      /* Usage (PID State report)                            */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, 0x02,    /* Report ID (2)                                       */
        0x09, 0x9F,    /* Usage (Device is Pause)                             */
        0x09, 0xA0,    /* Usage (Actuators Enabled)                           */
        0x09, 0xA4,    /* Usage (Safety Switch)                               */
        0x09, 0xA6,    /* Usage (Actuator Power)                              */
        0x09, 0x94,    /* Usage (Effect Playing)                              */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x25, 0x01,    /* Logical Maximum (1)                                 */
        0x35, 0x00,    /* Physical Minimum (0)                                */
        0x45, 0x01,    /* Physical Maximum (1)                                */
        0x75, 0x01,    /* Report Size (1)                                     */
        0x95, 0x05,    /* Report Count (5)                                    */
        0x81, 0x02,    /* Input (Variable)                                    */
        0x95, 0x03,    /* Report Count (3) — padding                          */
        0x81, 0x03,    /* Input (Constant, Variable)                          */
      0xC0,            /* End Collection (PID State)                          */

      /* ── Report 0x01 OUTPUT: Set Effect (SETEFREP, 131 bytes, NO closing C0) ──
       * Collection stays open; axes/direction/type-block content follows.      */
      0x09, 0x21,      /* Usage (Set Effect Report)                           */
      0xA1, 0x02,      /* Collection (Logical) — closed AFTER inline content  */
        0x85, 0x01,    /* Report ID (1)                                       */
        0x09, 0x22,    /* Usage (Effect Block Index)                          */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,    /* Physical Minimum (1)                                */
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x25,    /* Usage (Effect Type)                                 */
        0xA1, 0x02,    /* Collection (Logical)                                */
          0x09, 0x26,  /* ET Constant Force                                   */
          0x09, 0x27,  /* ET Ramp                                             */
          0x09, 0x30,  /* ET Square                                           */
          0x09, 0x31,  /* ET Sine                                             */
          0x09, 0x32,  /* ET Triangle                                         */
          0x09, 0x33,  /* ET Sawtooth Up                                      */
          0x09, 0x34,  /* ET Sawtooth Down                                    */
          0x09, 0x40,  /* ET Spring                                           */
          0x09, 0x41,  /* ET Damper                                           */
          0x09, 0x42,  /* ET Inertia                                          */
          0x09, 0x43,  /* ET Friction                                         */
          0x25, 0x0B,  /* Logical Maximum (11)                                */
          0x15, 0x01,  /* Logical Minimum (1)                                 */
          0x35, 0x01,  /* Physical Minimum (1)                                */
          0x45, 0x0B,  /* Physical Maximum (11)                               */
          0x75, 0x08,  /* Report Size (8)                                     */
          0x95, 0x01,  /* Report Count (1)                                    */
          0x91, 0x00,  /* Output (Array)                                      */
        0xC0,          /* End Collection (Effect Type)                        */
        0x09, 0x50,    /* Usage (Duration)                                    */
        0x09, 0x54,    /* Usage (Trigger Repeat Interval)                     */
        0x09, 0x51,    /* Usage (Sample Period)                               */
        0x09, 0xA7,    /* Usage (Start Delay)                                 */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x26, 0xFF, 0x7F,
        0x35, 0x00,    /* Physical Minimum (0)                                */
        0x46, 0xFF, 0x7F,
        0x66, 0x03, 0x10,  /* Unit (ms)                                       */
        0x55, 0xFD,    /* Unit Exponent (-3)                                  */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x04,    /* Report Count (4)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x55, 0x00,    /* Unit Exponent (0)                                   */
        0x66, 0x00, 0x00,
        0x09, 0x52,    /* Usage (Gain)                                        */
        0x15, 0x00,
        0x26, 0xFF, 0x00,
        0x35, 0x00,
        0x46, 0x10, 0x27,  /* Physical Maximum (10000)                        */
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x53,    /* Usage (Trigger Button)                              */
        0x15, 0x01,
        0x25, 0x08,
        0x35, 0x01,
        0x45, 0x08,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,    /* Output (Variable) — NO 0xC0 here                   */

      /* ── Inline: Axes Enable + Direction + Type Specific Block Offset (94 B) ─
       * Inside the Set Effect Logical Collection opened above.
       * The final 0xC0 closes that collection.                                */
        0x09, 0x55,          /* Usage (Axes Enable)                           */
        0xA1, 0x02,
          0x05, 0x01,        /* Usage Page (Generic Desktop)                  */
          0x09, 0x30,        /* Usage (X)                                     */
          0x15, 0x00,
          0x25, 0x00,
          0x75, 0x01,
          0x95, 0x01,
          0x91, 0x02,
        0xC0,                /* End Collection (Axes Enable)                  */
        0x05, 0x0F,          /* Usage Page (Physical Interface)               */
        0x09, 0x56,          /* Usage (Direction Enable)                      */
        0x95, 0x01,
        0x91, 0x02,
        0x95, 0x06,          /* Report Count (6) — padding                    */
        0x91, 0x03,
        0x09, 0x57,          /* Usage (Direction)                             */
        0xA1, 0x02,
          0x0B, 0x01, 0x00, 0x0A, 0x00,
          0x66, 0x14, 0x00,
          0x15, 0x00,
          0x27, 0xA0, 0x8C, 0x00, 0x00,  /* Logical Maximum (36000)           */
          0x35, 0x00,
          0x47, 0xA0, 0x8C, 0x00, 0x00,  /* Physical Maximum (36000)          */
          0x66, 0x00, 0x00,
          0x75, 0x10,
          0x95, 0x01,
          0x91, 0x02,
          0x55, 0x00,
          0x66, 0x00, 0x00,
        0xC0,                /* End Collection (Direction)                    */
        0x05, 0x0F,
        0x09, 0x58,          /* Usage (Type Specific Block Offset)            */
        0xA1, 0x02,
          0x0B, 0x01, 0x00, 0x0A, 0x00,
          0x26, 0xFD, 0x7F,  /* Logical Maximum (32765)                       */
          0x75, 0x10,
          0x95, 0x01,
          0x91, 0x02,
        0xC0,                /* End Collection (Type Specific Block Offset)   */
      0xC0,                  /* End Collection (Set Effect Report)            */

      /* ── Report 0x02 OUTPUT: Set Envelope (SETENVREP, 75 bytes) ── */
      0x09, 0x5A,
      0xA1, 0x02,
        0x85, 0x02,
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x5B,    /* Attack Level                                        */
        0x09, 0x5D,    /* Fade Level                                          */
        0x16, 0x00, 0x00,
        0x26, 0xFF, 0x7F,
        0x36, 0x00, 0x00,
        0x46, 0xFF, 0x7F,
        0x75, 0x10,
        0x95, 0x02,
        0x91, 0x02,
        0x09, 0x5C,    /* Attack Time                                         */
        0x09, 0x5E,    /* Fade Time                                           */
        0x66, 0x03, 0x10,
        0x55, 0xFD,
        0x27, 0xFF, 0x7F, 0x00, 0x00,
        0x47, 0xFF, 0x7F, 0x00, 0x00,
        0x75, 0x20,
        0x91, 0x02,    /* inherits Count (2)                                  */
        0x45, 0x00,
        0x66, 0x00, 0x00,
        0x55, 0x00,
      0xC0,

      /* ── Report 0x03 OUTPUT: Set Condition (inline 1-axis, 120 bytes) ── */
      0x09, 0x5F,
      0xA1, 0x02,
        0x85, 0x03,
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x23,    /* Parameter Block Offset                              */
        0x15, 0x00,
        0x25, 0x03,
        0x35, 0x00,
        0x45, 0x03,
        0x75, 0x06,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x58,    /* Type Specific Block Offset                          */
        0xA1, 0x02,
          0x0B, 0x01, 0x00, 0x0A, 0x00,
          0x75, 0x02,
          0x95, 0x01,
          0x91, 0x02,
        0xC0,
        0x16, 0x00, 0x80,
        0x26, 0xFF, 0x7F,
        0x36, 0x00, 0x80,
        0x46, 0xFF, 0x7F,
        0x09, 0x60,    /* CP Offset                                           */
        0x75, 0x10,
        0x95, 0x01,
        0x91, 0x02,
        0x36, 0x00, 0x80,
        0x46, 0xFF, 0x7F,
        0x09, 0x61,    /* Positive Coefficient                                */
        0x09, 0x62,    /* Negative Coefficient                                */
        0x95, 0x02,
        0x91, 0x02,
        0x16, 0x00, 0x00,
        0x26, 0xFF, 0x7F,
        0x36, 0x00, 0x00,
        0x46, 0xFF, 0x7F,
        0x09, 0x63,    /* Positive Saturation                                 */
        0x09, 0x64,    /* Negative Saturation                                 */
        0x75, 0x10,
        0x95, 0x02,
        0x91, 0x02,
        0x09, 0x65,    /* Dead Band                                           */
        0x46, 0xFF, 0x7F,
        0x95, 0x01,
        0x91, 0x02,
      0xC0,

      /* ── Report 0x04 OUTPUT: Set Periodic (SETPERIODICREP, 122 bytes) ── */
      0x09, 0x6E,
      0xA1, 0x02,
        0x85, 0x04,
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x70,    /* Magnitude                                           */
        0x16, 0x00, 0x00,
        0x26, 0xFF, 0x7F,
        0x36, 0x00, 0x00,
        0x26, 0xFF, 0x7F,  /* intentional 0x26 — matches OpenFFBoard          */
        0x75, 0x10,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x6F,    /* Offset                                              */
        0x16, 0x00, 0x80,
        0x26, 0xFF, 0x7F,
        0x36, 0x00, 0x80,
        0x46, 0xFF, 0x7F,
        0x95, 0x01,
        0x75, 0x10,
        0x91, 0x02,
        0x09, 0x71,    /* Phase                                               */
        0x66, 0x14, 0x00,
        0x55, 0xFE,
        0x15, 0x00,
        0x27, 0x9F, 0x8C, 0x00, 0x00,
        0x35, 0x00,
        0x47, 0x9F, 0x8C, 0x00, 0x00,
        0x75, 0x10,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x72,    /* Period                                              */
        0x15, 0x01,
        0x27, 0xFF, 0x7F, 0x00, 0x00,
        0x35, 0x01,
        0x47, 0xFF, 0x7F, 0x00, 0x00,
        0x66, 0x03, 0x10,
        0x55, 0xFD,
        0x75, 0x20,
        0x95, 0x01,
        0x91, 0x02,
        0x66, 0x00, 0x00,
        0x55, 0x00,
      0xC0,

      /* ── Report 0x05 OUTPUT: Set Constant Force (SETCFREP, 43 bytes) ── */
      0x09, 0x73,
      0xA1, 0x02,
        0x85, 0x05,
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x70,    /* Magnitude                                           */
        0x16, 0x01, 0x80,
        0x26, 0xFF, 0x7F,
        0x36, 0x01, 0x80,
        0x46, 0xFF, 0x7F,
        0x75, 0x10,
        0x95, 0x01,
        0x91, 0x02,
      0xC0,

      /* ── Report 0x06 OUTPUT: Set Ramp Force (SETRAMPREP, 45 bytes) ── */
      0x09, 0x74,
      0xA1, 0x02,
        0x85, 0x06,
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x75,    /* Ramp Start                                          */
        0x09, 0x76,    /* Ramp End                                            */
        0x16, 0x00, 0x80,
        0x26, 0xFF, 0x7F,
        0x36, 0x00, 0x80,
        0x46, 0xFF, 0x7F,
        0x75, 0x10,
        0x95, 0x02,
        0x91, 0x02,
      0xC0,

      /* ── Report 0x0A OUTPUT: Effect Operation (EFOPREP, 60 bytes) ── */
      0x05, 0x0F,
      0x09, 0x77,
      0xA1, 0x02,
        0x85, 0x0A,
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x78,
        0xA1, 0x02,
          0x09, 0x79,  /* Op Start                                            */
          0x09, 0x7A,  /* Op Start Solo                                       */
          0x09, 0x7B,  /* Op Stop                                             */
          0x15, 0x01,
          0x25, 0x03,
          0x75, 0x08,
          0x95, 0x01,
          0x91, 0x00,
        0xC0,
        0x09, 0x7C,    /* Loop Count — 8-bit, inherits Size(8) Count(1)       */
        0x15, 0x00,
        0x26, 0xFF, 0x00,
        0x35, 0x00,
        0x46, 0xFF, 0x00,
        0x91, 0x02,
      0xC0,

      /* ── Report 0x0B OUTPUT: Block Free (BLOCKFREEREP, 23 bytes) ── */
      0x09, 0x90,
      0xA1, 0x02,
        0x85, 0x0B,
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
      0xC0,

      /* ── Reports 0x0C+0x0D OUTPUT: Device Control + Device Gain (59 bytes) ─
       * Device Control: 8×1-bit Variable bit flags (OpenFFBoard style)         */
      0x09, 0x95,
      0xA1, 0x02,
        0x85, 0x0C,
        0x09, 0x96,
        0xA1, 0x02,
          0x09, 0x97,  /* DC Enable Actuators                                 */
          0x09, 0x98,  /* DC Disable Actuators                                */
          0x09, 0x99,  /* DC Stop All Effects                                 */
          0x09, 0x9A,  /* DC Device Reset                                     */
          0x09, 0x9B,  /* DC Device Pause                                     */
          0x09, 0x9C,  /* DC Device Continue                                  */
          0x15, 0x01,  /* Logical Minimum (1)                                 */
          0x25, 0x06,  /* Logical Maximum (6)                                 */
          0x75, 0x01,  /* Report Size (1)                                     */
          0x95, 0x08,  /* Report Count (8)                                    */
          0x91, 0x02,  /* Output (Variable)                                   */
        0xC0,
      0xC0,
      0x09, 0x7D,      /* Device Gain Report                                  */
      0xA1, 0x02,
        0x85, 0x0D,
        0x09, 0x7E,
        0x15, 0x00,
        0x26, 0xFF, 0x00,
        0x35, 0x00,
        0x46, 0x10, 0x27,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
      0xC0,

      /* ── Report 0x11 FEATURE: Create New Effect (NEWEFREP, 72 bytes) ── */
      0x09, 0xAB,
      0xA1, 0x02,
        0x85, 0x11,
        0x09, 0x25,
        0xA1, 0x02,
          0x09, 0x26,  0x09, 0x27,  0x09, 0x30,  0x09, 0x31,
          0x09, 0x32,  0x09, 0x33,  0x09, 0x34,  0x09, 0x40,
          0x09, 0x41,  0x09, 0x42,  0x09, 0x43,
          0x25, 0x0B,
          0x15, 0x01,
          0x35, 0x01,
          0x45, 0x0B,
          0x75, 0x08,
          0x95, 0x01,
          0xB1, 0x00,
        0xC0,
        0x05, 0x01,    /* Usage Page (Generic Desktop)                        */
        0x09, 0x3B,    /* Byte Count                                          */
        0x15, 0x00,
        0x26, 0xFF, 0x01,
        0x35, 0x00,
        0x46, 0xFF, 0x01,
        0x75, 0x0A,
        0x95, 0x01,
        0xB1, 0x02,
        0x75, 0x06,
        0xB1, 0x01,
      0xC0,

      /* ── Report 0x12 FEATURE: Block Load (BLOCKLOADREP, 72 bytes) ── */
      0x05, 0x0F,
      0x09, 0x89,
      0xA1, 0x02,
        0x85, 0x12,
        0x09, 0x22,
        0x25, FFB_MAX_EFFECTS,  /* Logical Max before Min — matches OpenFFBoard */
        0x15, 0x01,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0xB1, 0x02,
        0x09, 0x8B,
        0xA1, 0x02,
          0x09, 0x8C,  0x09, 0x8D,  0x09, 0x8E,
          0x15, 0x01,
          0x25, 0x03,
          0x35, 0x01,
          0x45, 0x03,
          0x75, 0x08,
          0x95, 0x01,
          0xB1, 0x00,
        0xC0,
        0x09, 0xAC,
        0x15, 0x00,
        0x27, 0xFF, 0xFF, 0x00, 0x00,
        0x35, 0x00,
        0x47, 0xFF, 0xFF, 0x00, 0x00,
        0x75, 0x10,
        0x95, 0x01,
        0xB1, 0x00,
      0xC0,

      /* ── Report 0x13 FEATURE: PID Pool (POOLREP, 67 bytes) ── */
      0x09, 0x7F,
      0xA1, 0x02,
        0x85, 0x13,
        0x09, 0x80,
        0x75, 0x10,
        0x95, 0x01,
        0x15, 0x00,
        0x35, 0x00,
        0x27, 0xFF, 0xFF, 0x00, 0x00,
        0x47, 0xFF, 0xFF, 0x00, 0x00,
        0xB1, 0x02,
        0x09, 0x83,
        0x26, 0xFF, 0x00,
        0x46, 0xFF, 0x00,
        0x75, 0x08,
        0x95, 0x01,
        0xB1, 0x02,
        0x09, 0xA9,    /* Device Managed Pool                                 */
        0x09, 0xAA,    /* Shared Parameter Blocks                             */
        0x75, 0x01,
        0x95, 0x02,
        0x15, 0x00,
        0x25, 0x01,
        0x35, 0x00,
        0x45, 0x01,
        0xB1, 0x02,
        0x75, 0x06,
        0x95, 0x01,
        0xB1, 0x03,
      0xC0,

    0xC0,              /* End Collection (TLC2 PID)                           */

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
