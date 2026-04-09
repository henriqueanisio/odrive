#include "usbd_hid_if.h"
#include <string.h>

/*
 * Combined HID Report Descriptor — two top-level Application Collections:
 *
 *   Collection 1: Generic Desktop / Joystick  (Report ID 0x01)
 *     Input 2 bytes — int16_t X axis
 *     → Enumerated by Windows as game controller (joy.cpl still works)
 *
 *   Collection 2: Vendor 0xFF00               (Report IDs 0x02, 0x03)
 *     Input   37 bytes — ODrive telemetry  (Report ID 0x02)
 *     Feature  6 bytes — ODrive commands   (Report ID 0x03)
 *     → Accessible via Python hid library (usage_page == 0xFF00)
 *
 * HID_REPORT_DESC_SIZE (usbd_hid.h) must equal the byte count below: 61
 */
__ALIGN_BEGIN uint8_t HID_ReportDesc[HID_REPORT_DESC_SIZE] __ALIGN_END = {

    /* ── Collection 1: Joystick (Generic Desktop) ── */
    0x05, 0x01,         /* Usage Page (Generic Desktop)          */
    0x09, 0x04,         /* Usage (Joystick)                      */
    0xA1, 0x01,         /* Collection (Application)              */
      0x85, HID_REPORT_ID_JOYSTICK,  /* Report ID (1)            */
      0x09, 0x30,       /* Usage (X)                             */
      0x16, 0x00, 0x80, /* Logical Minimum (-32768)              */
      0x26, 0xFF, 0x7F, /* Logical Maximum (+32767)              */
      0x75, 0x10,       /* Report Size (16 bits)                 */
      0x95, 0x01,       /* Report Count (1)                      */
      0x81, 0x02,       /* Input (Data, Variable, Absolute)      */
    0xC0,               /* End Collection                        */

    /* ── Collection 2: ODrive Vendor (0xFF00) ── */
    0x06, 0x00, 0xFF,   /* Usage Page (Vendor Defined 0xFF00)    */
    0x09, 0x01,         /* Usage (Vendor 1)                      */
    0xA1, 0x01,         /* Collection (Application)              */

      /* Input Report ID 2: Telemetry (37 bytes) */
      0x85, HID_REPORT_ID_TELEMETRY,       /* Report ID (2)      */
      0x09, 0x02,                          /* Usage (Vendor 2)   */
      0x15, 0x00,                          /* Logical Min (0)    */
      0x26, 0xFF, 0x00,                    /* Logical Max (255)  */
      0x75, 0x08,                          /* Report Size (8)    */
      0x95, HID_TELEMETRY_PAYLOAD_SIZE,    /* Report Count (37)  */
      0x81, 0x02,                          /* Input              */

      /* Feature Report ID 3: Command (6 bytes) */
      0x85, HID_REPORT_ID_COMMAND,         /* Report ID (3)      */
      0x09, 0x03,                          /* Usage (Vendor 3)   */
      0x15, 0x00,                          /* Logical Min (0)    */
      0x26, 0xFF, 0x00,                    /* Logical Max (255)  */
      0x75, 0x08,                          /* Report Size (8)    */
      0x95, HID_COMMAND_PAYLOAD_SIZE,      /* Report Count (6)   */
      0xB1, 0x02,                          /* Feature            */

    0xC0,               /* End Collection                        */
};
/* sizeof(HID_ReportDesc) == 61  →  HID_REPORT_DESC_SIZE must be 61 */

/* ── Joystick send (Report ID 0x01) ─────────────────────────────────────────── */
uint8_t HID_Joystick_Send(int16_t value)
{
    uint8_t report[3];   /* report_id + 2 data bytes */
    report[0] = HID_REPORT_ID_JOYSTICK;
    report[1] = (uint8_t)((uint16_t)value & 0xFF);
    report[2] = (uint8_t)((uint16_t)value >> 8);
    return USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}

/* ── Telemetry send (Report ID 0x02) ────────────────────────────────────────── */
uint8_t HID_ODrive_SendTelemetry(const HID_TelemetryPayload_t *payload)
{
    uint8_t report[1 + HID_TELEMETRY_PAYLOAD_SIZE];
    report[0] = HID_REPORT_ID_TELEMETRY;
    memcpy(&report[1], payload, HID_TELEMETRY_PAYLOAD_SIZE);
    return USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}

/* ── Weak default command handler ────────────────────────────────────────────
 *
 * Override this in your application (e.g. communication.cpp):
 *
 *   extern "C" void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *cmd) {
 *       switch (cmd->cmd_id) {
 *           case HID_CMD_SET_AXIS_STATE:
 *               odrv.axis0.requested_state_ = (uint8_t)cmd->param;  break;
 *           case HID_CMD_CLEAR_ERRORS:
 *               odrv.axis0.error_ = 0;                              break;
 *           case HID_CMD_SAVE_CONFIG:
 *               odrv.save_configuration();                           break;
 *           case HID_CMD_REBOOT:
 *               NVIC_SystemReset();                                  break;
 *           case HID_CMD_ENTER_DFU:
 *               odrv.enter_dfu_mode();                               break;
 *           case HID_CMD_SET_INPUT_POS:
 *               odrv.axis0.controller_.input_pos_ = cmd->param;     break;
 *           case HID_CMD_SET_CURRENT_LIM:
 *               odrv.axis0.motor_.config_.current_lim = cmd->param; break;
 *       }
 *   }
 * ─────────────────────────────────────────────────────────────────────────── */
__attribute__((weak)) void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *cmd)
{
    (void)cmd;
}
