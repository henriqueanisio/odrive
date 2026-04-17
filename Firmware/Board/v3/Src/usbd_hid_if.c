#include "usbd_hid_if.h"
#include "ffb_pid.h"
#include <string.h>
#include "hid_queue.h"

/* ── Combined HID Report Descriptor ──────────────────────────────────────────
 *
 * Structure matches OpenFFBoard reference (usb_hid_1ffb_desc.c) exactly:
 *   All FFB/PID reports in one Joystick Application Collection (TLC1).
 *   Vendor telemetry/command/config in a separate Vendor Collection (TLC2).
 *
 * TLC1 (1043 bytes):
 *   Report 0x01 (IN,  2 B)  Joystick X axis
 *   Report 0x02 (IN,  1 B)  PID State
 *   Report 0x01 (OUT,12 B)  Set Effect
 *   Report 0x02 (OUT, 9 B)  Set Envelope
 *   Report 0x03 (OUT,12 B)  Set Condition
 *   Report 0x04 (OUT,11 B)  Set Periodic
 *   Report 0x05 (OUT, 3 B)  Set Constant Force
 *   Report 0x06 (OUT, 5 B)  Set Ramp
 *   Report 0x0A (OUT, 4 B)  Effect Operation
 *   Report 0x0B (OUT, 1 B)  Block Free
 *   Report 0x0C (OUT, 1 B)  Device Control
 *   Report 0x0D (OUT, 1 B)  Device Gain
 *   Report 0x11 (FEAT,3 B)  Create New Effect
 *   Report 0x12 (FEAT,4 B)  Block Load
 *   Report 0x13 (FEAT,4 B)  PID Pool
 *
 * TLC2 (53 bytes):
 *   Report 0x20 (IN, 52 B)  Telemetry
 *   Report 0x22 (IN,  6 B)  Config Response
 *   Report 0x21 (FEAT,8 B)  Command
 *
 * Total = 1096 bytes = HID_REPORT_DESC_SIZE in usbd_hid.h.
 * ─────────────────────────────────────────────────────────────────────────── */
__ALIGN_BEGIN uint8_t HID_ReportDesc[HID_REPORT_DESC_SIZE] __ALIGN_END = {

    /* ═══════════════════════════════════════════════════════════════════════
     * TLC1: Generic Desktop / Joystick + PID  (1043 bytes)
     * ═══════════════════════════════════════════════════════════════════════ */
    0x05, 0x01,        /* Usage Page (Generic Desktop)                        */
    0x09, 0x04,        /* Usage (Joystick)                                    */
    0xA1, 0x01,        /* Collection (Application)                            */

      /* ── Report 0x01 INPUT: Joystick X (16 bytes) ── */
      0x85, 0x01,      /* Report ID (1)                                       */
      0x09, 0x30,      /* Usage (X)                                           */
      0x16, 0x00, 0x80,/* Logical Minimum (-32768)                            */
      0x26, 0xFF, 0x7F,/* Logical Maximum (32767)                             */
      0x75, 0x10,      /* Report Size (16)                                    */
      0x95, 0x01,      /* Report Count (1)                                    */
      0x81, 0x02,      /* Input (Variable, Absolute)                          */

      /* ── PID State Report 0x02 INPUT (STATEREP, 37 bytes incl. page switch) ── */
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
       * NOTE: collection stays open; axes/direction content follows below.     */
      0x09, 0x21,      /* Usage (Set Effect Report)                           */
      0xA1, 0x02,      /* Collection (Logical) — closed AFTER inline content  */
        0x85, 0x01,    /* Report ID (1)                                       */
        0x09, 0x22,    /* Usage (Effect Block Index)                          */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, FFB_MAX_EFFECTS, /* Logical Maximum                             */
        0x35, 0x01,    /* Physical Minimum (1)                                */
        0x45, FFB_MAX_EFFECTS, /* Physical Maximum                            */
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
        0x26, 0xFF, 0x7F, /* Logical Maximum (32767)                          */
        0x35, 0x00,    /* Physical Minimum (0)                                */
        0x46, 0xFF, 0x7F, /* Physical Maximum (32767)                         */
        0x66, 0x03, 0x10, /* Unit (ms)                                        */
        0x55, 0xFD,    /* Unit Exponent (-3)                                  */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x04,    /* Report Count (4)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x55, 0x00,    /* Unit Exponent (0)                                   */
        0x66, 0x00, 0x00, /* Unit (0)                                         */
        0x09, 0x52,    /* Usage (Gain)                                        */
        0x15, 0x00,    /* Logical Minimum (0)                                 */
        0x26, 0xFF, 0x00, /* Logical Maximum (255)                            */
        0x35, 0x00,    /* Physical Minimum (0)                                */
        0x46, 0x10, 0x27, /* Physical Maximum (10000)                         */
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x53,    /* Usage (Trigger Button)                              */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, 0x08,    /* Logical Maximum (8)                                 */
        0x35, 0x01,    /* Physical Minimum (1)                                */
        0x45, 0x08,    /* Physical Maximum (8)                                */
        0x75, 0x08,    /* Report Size (8)                                     */
        0x95, 0x01,    /* Report Count (1)                                    */
        0x91, 0x02,    /* Output (Variable) — NO 0xC0 here; collection stays open */

      /* ── Inline: Axes Enable + Direction + Type Specific Block Offset (94 bytes)
       * This content is INSIDE the Set Effect collection opened above.
       * The final 0xC0 below closes that collection.                          */
        0x09, 0x55,          /* Usage (Axes Enable)                           */
        0xA1, 0x02,          /* Collection (Logical)                          */
          0x05, 0x01,        /* Usage Page (Generic Desktop)                  */
          0x09, 0x30,        /* Usage (X)                                     */
          0x15, 0x00,        /* Logical Minimum (0)                           */
          0x25, 0x00,        /* Logical Maximum (0)                           */
          0x75, 0x01,        /* Report Size (1)                               */
          0x95, 0x01,        /* Report Count (1)                              */
          0x91, 0x02,        /* Output (Variable)                             */
        0xC0,                /* End Collection (Axes Enable)                  */
        0x05, 0x0F,          /* Usage Page (Physical Interface)               */
        0x09, 0x56,          /* Usage (Direction Enable)                      */
        0x95, 0x01,          /* Report Count (1)                              */
        0x91, 0x02,          /* Output (Variable)                             */
        0x95, 0x06,          /* Report Count (6) — padding                    */
        0x91, 0x03,          /* Output (Constant, Variable)                   */
        0x09, 0x57,          /* Usage (Direction)                             */
        0xA1, 0x02,          /* Collection (Logical)                          */
          0x0B, 0x01, 0x00, 0x0A, 0x00, /* Usage (Ordinals: Instance 1)      */
          0x66, 0x14, 0x00,  /* Unit (Eng Rotation)                           */
          0x15, 0x00,        /* Logical Minimum (0)                           */
          0x27, 0xA0, 0x8C, 0x00, 0x00, /* Logical Maximum (36000)           */
          0x35, 0x00,        /* Physical Minimum (0)                          */
          0x47, 0xA0, 0x8C, 0x00, 0x00, /* Physical Maximum (36000)          */
          0x66, 0x00, 0x00,  /* Unit (0)                                      */
          0x75, 0x10,        /* Report Size (16)                              */
          0x95, 0x01,        /* Report Count (1)                              */
          0x91, 0x02,        /* Output (Variable)                             */
          0x55, 0x00,        /* Unit Exponent (0)                             */
          0x66, 0x00, 0x00,  /* Unit (0)                                      */
        0xC0,                /* End Collection (Direction)                    */
        0x05, 0x0F,          /* Usage Page (Physical Interface)               */
        0x09, 0x58,          /* Usage (Type Specific Block Offset)            */
        0xA1, 0x02,          /* Collection (Logical)                          */
          0x0B, 0x01, 0x00, 0x0A, 0x00, /* Usage (Ordinals: Instance 1)      */
          0x26, 0xFD, 0x7F,  /* Logical Maximum (32765)                       */
          0x75, 0x10,        /* Report Size (16)                              */
          0x95, 0x01,        /* Report Count (1)                              */
          0x91, 0x02,        /* Output (Data, Var, Abs)                       */
        0xC0,                /* End Collection (Type Specific Block Offset)   */
      0xC0,                  /* End Collection (Set Effect Report)            */

      /* ── Report 0x02 OUTPUT: Set Envelope (SETENVREP, 75 bytes) ── */
      0x09, 0x5A,      /* Usage (Set Envelope Report)                         */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, 0x02,    /* Report ID (2)                                       */
        0x09, 0x22,    /* Usage (Effect Block Index)                          */
        0x15, 0x01,    /* Logical Minimum (1)                                 */
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x5B,    /* Usage (Attack Level)                                */
        0x09, 0x5D,    /* Usage (Fade Level)                                  */
        0x16, 0x00, 0x00,  /* Logical Minimum (0)                             */
        0x26, 0xFF, 0x7F,  /* Logical Maximum (32767)                         */
        0x36, 0x00, 0x00,  /* Physical Minimum (0)                            */
        0x46, 0xFF, 0x7F,  /* Physical Maximum (32767)                        */
        0x75, 0x10,    /* Report Size (16)                                    */
        0x95, 0x02,    /* Report Count (2)                                    */
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x5C,    /* Usage (Attack Time)                                 */
        0x09, 0x5E,    /* Usage (Fade Time)                                   */
        0x66, 0x03, 0x10,  /* Unit (ms)                                       */
        0x55, 0xFD,    /* Unit Exponent (-3)                                  */
        0x27, 0xFF, 0x7F, 0x00, 0x00, /* Logical Maximum (32767 as 4-byte)   */
        0x47, 0xFF, 0x7F, 0x00, 0x00, /* Physical Maximum (32767 as 4-byte)  */
        0x75, 0x20,    /* Report Size (32)                                    */
        0x91, 0x02,    /* Output (Variable) — inherits Count (2)              */
        0x45, 0x00,    /* Physical Maximum (0)                                */
        0x66, 0x00, 0x00, /* Unit (0)                                         */
        0x55, 0x00,    /* Unit Exponent (0)                                   */
      0xC0,            /* End Collection (Set Envelope)                       */

      /* ── Report 0x03 OUTPUT: Set Condition (inline 1-axis, 120 bytes) ── */
      0x09, 0x5F,      /* Usage (Set Condition Report)                        */
      0xA1, 0x02,      /* Collection (Logical)                                */
        0x85, 0x03,    /* Report ID (3)                                       */
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,    /* Output (Variable)                                   */
        0x09, 0x23,    /* Usage (Parameter Block Offset)                      */
        0x15, 0x00,
        0x25, 0x03,
        0x35, 0x00,
        0x45, 0x03,
        0x75, 0x06,    /* Report Size (6)                                     */
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x58,    /* Usage (Type Specific Block Offset)                  */
        0xA1, 0x02,
          0x0B, 0x01, 0x00, 0x0A, 0x00, /* Usage (Ordinals: Instance 1)      */
          0x75, 0x02,  /* Report Size (2)                                     */
          0x95, 0x01,
          0x91, 0x02,
        0xC0,          /* End Collection                                      */
        0x16, 0x00, 0x80, /* Logical Minimum (-32768)                         */
        0x26, 0xFF, 0x7F, /* Logical Maximum (32767)                          */
        0x36, 0x00, 0x80, /* Physical Minimum (-32768)                        */
        0x46, 0xFF, 0x7F, /* Physical Maximum (32767)                         */
        0x09, 0x60,    /* Usage (CP Offset)                                   */
        0x75, 0x10,
        0x95, 0x01,
        0x91, 0x02,
        0x36, 0x00, 0x80, /* Physical Minimum (-32768)                        */
        0x46, 0xFF, 0x7F,
        0x09, 0x61,    /* Usage (Positive Coefficient)                        */
        0x09, 0x62,    /* Usage (Negative Coefficient)                        */
        0x95, 0x02,
        0x91, 0x02,
        0x16, 0x00, 0x00, /* Logical Minimum (0)                              */
        0x26, 0xFF, 0x7F, /* Logical Maximum (32767)                          */
        0x36, 0x00, 0x00,
        0x46, 0xFF, 0x7F,
        0x09, 0x63,    /* Usage (Positive Saturation)                         */
        0x09, 0x64,    /* Usage (Negative Saturation)                         */
        0x75, 0x10,
        0x95, 0x02,
        0x91, 0x02,
        0x09, 0x65,    /* Usage (Dead Band)                                   */
        0x46, 0xFF, 0x7F,
        0x95, 0x01,
        0x91, 0x02,
      0xC0,            /* End Collection (Set Condition)                      */

      /* ── Report 0x04 OUTPUT: Set Periodic (SETPERIODICREP, 122 bytes) ── */
      0x09, 0x6E,      /* Usage (Set Periodic Report)                         */
      0xA1, 0x02,
        0x85, 0x04,    /* Report ID (4)                                       */
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x70,    /* Usage (Magnitude)                                   */
        0x16, 0x00, 0x00,
        0x26, 0xFF, 0x7F,  /* Logical Maximum (32767)                         */
        0x36, 0x00, 0x00,
        0x26, 0xFF, 0x7F,  /* NOTE: 0x26 intentional — matches OpenFFBoard    */
        0x75, 0x10,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x6F,    /* Usage (Offset)                                      */
        0x16, 0x00, 0x80,
        0x26, 0xFF, 0x7F,
        0x36, 0x00, 0x80,
        0x46, 0xFF, 0x7F,
        0x95, 0x01,
        0x75, 0x10,
        0x91, 0x02,
        0x09, 0x71,    /* Usage (Phase)                                       */
        0x66, 0x14, 0x00,  /* Unit (Eng Rotation)                             */
        0x55, 0xFE,    /* Unit Exponent (-2)                                  */
        0x15, 0x00,
        0x27, 0x9F, 0x8C, 0x00, 0x00, /* Logical Maximum (35999)             */
        0x35, 0x00,
        0x47, 0x9F, 0x8C, 0x00, 0x00, /* Physical Maximum (35999)            */
        0x75, 0x10,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x72,    /* Usage (Period)                                      */
        0x15, 0x01,
        0x27, 0xFF, 0x7F, 0x00, 0x00, /* Logical Maximum (32767)             */
        0x35, 0x01,
        0x47, 0xFF, 0x7F, 0x00, 0x00, /* Physical Maximum (32767)            */
        0x66, 0x03, 0x10,  /* Unit (ms)                                       */
        0x55, 0xFD,
        0x75, 0x20,    /* Report Size (32)                                    */
        0x95, 0x01,
        0x91, 0x02,
        0x66, 0x00, 0x00,  /* Unit (0)                                        */
        0x55, 0x00,    /* Unit Exponent (0)                                   */
      0xC0,            /* End Collection (Set Periodic)                       */

      /* ── Report 0x05 OUTPUT: Set Constant Force (SETCFREP, 43 bytes) ── */
      0x09, 0x73,      /* Usage (Set Constant Force Report)                   */
      0xA1, 0x02,
        0x85, 0x05,    /* Report ID (5)                                       */
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x70,    /* Usage (Magnitude)                                   */
        0x16, 0x01, 0x80,  /* Logical Minimum (-32767)                        */
        0x26, 0xFF, 0x7F,  /* Logical Maximum (32767)                         */
        0x36, 0x01, 0x80,
        0x46, 0xFF, 0x7F,
        0x75, 0x10,
        0x95, 0x01,
        0x91, 0x02,
      0xC0,            /* End Collection (Set Constant Force)                 */

      /* ── Report 0x06 OUTPUT: Set Ramp Force (SETRAMPREP, 45 bytes) ── */
      0x09, 0x74,      /* Usage (Set Ramp Force Report)                       */
      0xA1, 0x02,
        0x85, 0x06,    /* Report ID (6)                                       */
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x75,    /* Usage (Ramp Start)                                  */
        0x09, 0x76,    /* Usage (Ramp End)                                    */
        0x16, 0x00, 0x80,  /* Logical Minimum (-32768)                        */
        0x26, 0xFF, 0x7F,
        0x36, 0x00, 0x80,
        0x46, 0xFF, 0x7F,
        0x75, 0x10,
        0x95, 0x02,
        0x91, 0x02,
      0xC0,            /* End Collection (Set Ramp)                           */

      /* ── Report 0x0A OUTPUT: Effect Operation (EFOPREP, 60 bytes) ── */
      0x05, 0x0F,      /* Usage Page (Physical Interface Device)              */
      0x09, 0x77,      /* Usage (Effect Operation Report)                     */
      0xA1, 0x02,
        0x85, 0x0A,    /* Report ID (0x0A)                                    */
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
        0x09, 0x78,    /* Usage (Effect Operation)                            */
        0xA1, 0x02,
          0x09, 0x79,  /* Op Effect Start                                     */
          0x09, 0x7A,  /* Op Effect Start Solo                                */
          0x09, 0x7B,  /* Op Effect Stop                                      */
          0x15, 0x01,
          0x25, 0x03,
          0x75, 0x08,
          0x95, 0x01,
          0x91, 0x00,  /* Output (Array)                                      */
        0xC0,
        0x09, 0x7C,    /* Usage (Loop Count)                                  */
        0x15, 0x00,
        0x26, 0xFF, 0x00, /* Logical Maximum (255) — 8-bit, inherits Size(8)  */
        0x35, 0x00,
        0x46, 0xFF, 0x00,
        0x91, 0x02,    /* Output (Variable) — NO Report Size/Count: inherits  */
      0xC0,            /* End Collection (Effect Operation)                   */

      /* ── Report 0x0B OUTPUT: Block Free (BLOCKFREEREP, 23 bytes) ── */
      0x09, 0x90,      /* Usage (PID Block Free Report)                       */
      0xA1, 0x02,
        0x85, 0x0B,    /* Report ID (0x0B)                                    */
        0x09, 0x22,
        0x15, 0x01,
        0x25, FFB_MAX_EFFECTS,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
      0xC0,            /* End Collection (Block Free)                         */

      /* ── Reports 0x0C + 0x0D OUTPUT: Device Control + Device Gain (59 bytes) ─
       * Device Control uses 8 × 1-bit Variable fields (bit flags per OpenFFBoard)
       * Bit 0=Enable, 1=Disable, 2=Stop, 3=Reset, 4=Pause, 5=Continue        */
      0x09, 0x95,      /* Usage (PID Device Control)                          */
      0xA1, 0x02,
        0x85, 0x0C,    /* Report ID (0x0C)                                    */
        0x09, 0x96,
        0xA1, 0x02,
          0x09, 0x97,  /* DC Enable Actuators                                 */
          0x09, 0x98,  /* DC Disable Actuators                                */
          0x09, 0x99,  /* DC Stop All Effects                                 */
          0x09, 0x9A,  /* DC Device Reset                                     */
          0x09, 0x9B,  /* DC Device Pause                                     */
          0x09, 0x9C,  /* DC Device Continue                                  */
          0x15, 0x01,
          0x25, 0x06,
          0x75, 0x01,  /* Report Size (1) — bit flags                         */
          0x95, 0x08,  /* Report Count (8)                                    */
          0x91, 0x02,  /* Output (Variable)                                   */
        0xC0,
      0xC0,            /* End Collection (Device Control)                     */
      0x09, 0x7D,      /* Usage (Device Gain Report)                          */
      0xA1, 0x02,
        0x85, 0x0D,    /* Report ID (0x0D)                                    */
        0x09, 0x7E,    /* Usage (Device Gain)                                 */
        0x15, 0x00,
        0x26, 0xFF, 0x00,
        0x35, 0x00,
        0x46, 0x10, 0x27,  /* Physical Maximum (10000)                        */
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x02,
      0xC0,            /* End Collection (Device Gain)                        */

      /* ── Report 0x11 FEATURE: Create New Effect (NEWEFREP, 72 bytes) ── */
      0x09, 0xAB,      /* Usage (Create New Effect Report)                    */
      0xA1, 0x02,
        0x85, 0x11,    /* Report ID (0x11)                                    */
        0x09, 0x25,    /* Usage (Effect Type)                                 */
        0xA1, 0x02,
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
          0x25, 0x0B,
          0x15, 0x01,
          0x35, 0x01,
          0x45, 0x0B,
          0x75, 0x08,
          0x95, 0x01,
          0xB1, 0x00,  /* Feature (Array)                                     */
        0xC0,          /* End Collection (Effect Type)                        */
        0x05, 0x01,    /* Usage Page (Generic Desktop)                        */
        0x09, 0x3B,    /* Usage (Byte Count)                                  */
        0x15, 0x00,
        0x26, 0xFF, 0x01,  /* Logical Maximum (511)                           */
        0x35, 0x00,
        0x46, 0xFF, 0x01,  /* Physical Maximum (511)                          */
        0x75, 0x0A,    /* Report Size (10)                                    */
        0x95, 0x01,
        0xB1, 0x02,    /* Feature (Variable)                                  */
        0x75, 0x06,    /* Report Size (6) — padding                           */
        0xB1, 0x01,    /* Feature (Constant)                                  */
      0xC0,            /* End Collection (Create New Effect)                  */

      /* ── Report 0x12 FEATURE: Block Load (BLOCKLOADREP, 72 bytes) ── */
      0x05, 0x0F,      /* Usage Page (Physical Interface Device)              */
      0x09, 0x89,      /* Usage (Block Load Report)                           */
      0xA1, 0x02,
        0x85, 0x12,    /* Report ID (0x12)                                    */
        0x09, 0x22,
        0x25, FFB_MAX_EFFECTS, /* Logical Max BEFORE Min — matches OpenFFBoard */
        0x15, 0x01,
        0x35, 0x01,
        0x45, FFB_MAX_EFFECTS,
        0x75, 0x08,
        0x95, 0x01,
        0xB1, 0x02,    /* Feature (Variable)                                  */
        0x09, 0x8B,    /* Usage (Block Load Status)                           */
        0xA1, 0x02,
          0x09, 0x8C,  /* Block Load Success                                  */
          0x09, 0x8D,  /* Block Load Full                                     */
          0x09, 0x8E,  /* Block Load Error                                    */
          0x15, 0x01,
          0x25, 0x03,
          0x35, 0x01,
          0x45, 0x03,
          0x75, 0x08,
          0x95, 0x01,
          0xB1, 0x00,  /* Feature (Array)                                     */
        0xC0,
        0x09, 0xAC,    /* Usage (Pool Available)                              */
        0x15, 0x00,
        0x27, 0xFF, 0xFF, 0x00, 0x00, /* Logical Maximum (65535)             */
        0x35, 0x00,
        0x47, 0xFF, 0xFF, 0x00, 0x00, /* Physical Maximum (65535)            */
        0x75, 0x10,
        0x95, 0x01,
        0xB1, 0x00,    /* Feature (Array)                                     */
      0xC0,            /* End Collection (Block Load)                         */

      /* ── Report 0x13 FEATURE: PID Pool (POOLREP, 67 bytes) ── */
      0x09, 0x7F,      /* Usage (PID Pool Report)                             */
      0xA1, 0x02,
        0x85, 0x13,    /* Report ID (0x13)                                    */
        0x09, 0x80,    /* Usage (RAM Pool Size)                               */
        0x75, 0x10,
        0x95, 0x01,
        0x15, 0x00,
        0x35, 0x00,
        0x27, 0xFF, 0xFF, 0x00, 0x00,
        0x47, 0xFF, 0xFF, 0x00, 0x00,
        0xB1, 0x02,    /* Feature (Variable)                                  */
        0x09, 0x83,    /* Usage (Simultaneous Effects Max)                    */
        0x26, 0xFF, 0x00,
        0x46, 0xFF, 0x00,
        0x75, 0x08,
        0x95, 0x01,
        0xB1, 0x02,
        0x09, 0xA9,    /* Usage (Device Managed Pool)                         */
        0x09, 0xAA,    /* Usage (Shared Parameter Blocks)                     */
        0x75, 0x01,
        0x95, 0x02,
        0x15, 0x00,
        0x25, 0x01,
        0x35, 0x00,
        0x45, 0x01,
        0xB1, 0x02,
        0x75, 0x06,    /* Report Size (6) — padding                           */
        0x95, 0x01,
        0xB1, 0x03,    /* Feature (Constant, Variable)                        */
      0xC0,            /* End Collection (PID Pool)                           */

    0xC0,              /* End Collection (Joystick + PID Application)         */

    /* ═══════════════════════════════════════════════════════════════════════
     * TLC2: Vendor 0xFF00  (53 bytes)
     * ═══════════════════════════════════════════════════════════════════════ */
    0x06, 0x00, 0xFF,  /* Usage Page (Vendor Defined 0xFF00)                  */
    0x09, 0x01,        /* Usage (Vendor 1)                                    */
    0xA1, 0x01,        /* Collection (Application)                            */

      0x85, HID_REPORT_ID_TELEMETRY,    /* Report ID (0x20)                  */
      0x09, 0x02,
      0x15, 0x00,
      0x26, 0xFF, 0x00,
      0x75, 0x08,
      0x95, HID_TELEMETRY_PAYLOAD_SIZE,
      0x81, 0x02,      /* Input (Variable)                                    */

      0x85, HID_REPORT_ID_CONFIG_RESP,  /* Report ID (0x22)                  */
      0x09, 0x04,
      0x15, 0x00,
      0x26, 0xFF, 0x00,
      0x75, 0x08,
      0x95, HID_CONFIG_RESP_PAYLOAD_SIZE,
      0x81, 0x02,      /* Input (Variable)                                    */

      0x85, HID_REPORT_ID_COMMAND,      /* Report ID (0x21)                  */
      0x09, 0x03,
      0x15, 0x00,
      0x26, 0xFF, 0x00,
      0x75, 0x08,
      0x95, HID_COMMAND_PAYLOAD_SIZE,
      0xB1, 0x02,      /* Feature (Variable)                                  */

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
