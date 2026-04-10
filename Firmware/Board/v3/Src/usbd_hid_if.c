#include "usbd_hid_if.h"
#include <string.h>

/* ── Combined HID Report Descriptor ──────────────────────────────────────────
 *
 * Two Application Collections:
 *
 *   1. Generic Desktop / Joystick  (Report ID 0x01, 2 bytes Input)
 *      → Windows game controller driver, shows in joy.cpl
 *
 *   2. Vendor 0xFF00               (Reports 0x02, 0x03, 0x04)
 *      → Accessible via Python hid library (filter usage_page == 0xFF00)
 *         0x02 Input   37 bytes  Telemetry
 *         0x03 Feature  8 bytes  Commands (host → device)
 *         0x04 Input    6 bytes  Config Response
 *
 * Total descriptor size: 23 + 53 = 76 bytes
 * HID_REPORT_DESC_SIZE (usbd_hid.h) must equal 76.
 * ─────────────────────────────────────────────────────────────────────────── */
__ALIGN_BEGIN uint8_t HID_ReportDesc[HID_REPORT_DESC_SIZE] __ALIGN_END = {

    /* ── Collection 1: Joystick ── */
    0x05, 0x01,          /* Usage Page (Generic Desktop)                      */
    0x09, 0x04,          /* Usage (Joystick)                                  */
    0xA1, 0x01,          /* Collection (Application)                          */
      0x85, HID_REPORT_ID_JOYSTICK, /* Report ID 1                           */
      0x09, 0x30,        /* Usage (X axis)                                    */
      0x16, 0x00, 0x80,  /* Logical Minimum (-32768)                          */
      0x26, 0xFF, 0x7F,  /* Logical Maximum (+32767)                          */
      0x75, 0x10,        /* Report Size (16 bits)                             */
      0x95, 0x01,        /* Report Count (1)                                  */
      0x81, 0x02,        /* Input (Data, Variable, Absolute)                  */
    0xC0,                /* End Collection                                    */

    /* ── Collection 2: ODrive Vendor ── */
    0x06, 0x00, 0xFF,    /* Usage Page (Vendor Defined 0xFF00)                */
    0x09, 0x01,          /* Usage (Vendor 1)                                  */
    0xA1, 0x01,          /* Collection (Application)                          */

      /* Report ID 2: Telemetry — 37 bytes Input */
      0x85, HID_REPORT_ID_TELEMETRY,          /* Report ID 2                 */
      0x09, 0x02,                             /* Usage (Vendor 2)            */
      0x15, 0x00,                             /* Logical Min (0)             */
      0x26, 0xFF, 0x00,                       /* Logical Max (255)           */
      0x75, 0x08,                             /* Report Size (8 bits)        */
      0x95, HID_TELEMETRY_PAYLOAD_SIZE,       /* Report Count (37)           */
      0x81, 0x02,                             /* Input                       */

      /* Report ID 4: Config Response — 6 bytes Input */
      0x85, HID_REPORT_ID_CONFIG_RESP,        /* Report ID 4                 */
      0x09, 0x04,                             /* Usage (Vendor 4)            */
      0x15, 0x00,                             /* Logical Min (0)             */
      0x26, 0xFF, 0x00,                       /* Logical Max (255)           */
      0x75, 0x08,                             /* Report Size (8 bits)        */
      0x95, HID_CONFIG_RESP_PAYLOAD_SIZE,     /* Report Count (6)            */
      0x81, 0x02,                             /* Input                       */

      /* Report ID 3: Command — 8 bytes Feature */
      0x85, HID_REPORT_ID_COMMAND,            /* Report ID 3                 */
      0x09, 0x03,                             /* Usage (Vendor 3)            */
      0x15, 0x00,                             /* Logical Min (0)             */
      0x26, 0xFF, 0x00,                       /* Logical Max (255)           */
      0x75, 0x08,                             /* Report Size (8 bits)        */
      0x95, HID_COMMAND_PAYLOAD_SIZE,         /* Report Count (8)            */
      0xB1, 0x02,                             /* Feature                     */

    0xC0,                /* End Collection                                    */
};
/* ─────────────────────────────────────────────────────────────────────────── */

/* ── HID_Joystick_Send ───────────────────────────────────────────────────── */
uint8_t HID_Joystick_Send(int16_t value)
{
    uint8_t report[3];   /* report_id + 2 data bytes */
    report[0] = HID_REPORT_ID_JOYSTICK;
    report[1] = (uint8_t)((uint16_t)value & 0xFFU);
    report[2] = (uint8_t)((uint16_t)value >> 8U);
    return USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}


/* ── HID_ODrive_SendTelemetry ────────────────────────────────────────────── */
uint8_t HID_ODrive_SendTelemetry(const HID_TelemetryPayload_t *payload)
{
    uint8_t report[1U + HID_TELEMETRY_PAYLOAD_SIZE];
    report[0] = HID_REPORT_ID_TELEMETRY;
    memcpy(&report[1], payload, HID_TELEMETRY_PAYLOAD_SIZE);
    return USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}

/* ── HID_ODrive_SendConfigResponse ──────────────────────────────────────── */
uint8_t HID_ODrive_SendConfigResponse(uint16_t param_id, float value)
{
    uint8_t report[1 + 2 + 4];

    report[0] = HID_REPORT_ID_CONFIG_RESP;
    memcpy(&report[1], &param_id, 2);
    memcpy(&report[3], &value, 4);

    return USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}

/* ── HID_ODrive_ProcessCommand (weak default) ────────────────────────────────
 * Override in hid_app_integration.cpp.
 *
 * Example:
 *   extern "C" void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *cmd){
 *       protocol_dispatch((const HID_Command_t *)cmd);
 *   }
 * ─────────────────────────────────────────────────────────────────────────── */
__attribute__((weak)) void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *cmd)
{
    (void)cmd;
}
