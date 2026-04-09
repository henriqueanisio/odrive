#include "usbd_hid_if.h"

/*
 * HID Report Descriptor — single-axis joystick (16-bit signed X).
 * One report = 2 bytes (int16_t).
 * Size must match HID_REPORT_DESC_SIZE defined in usbd_hid.h.
 */
__ALIGN_BEGIN uint8_t HID_ReportDesc[HID_REPORT_DESC_SIZE] __ALIGN_END = {
    0x05, 0x01,        /* Usage Page (Generic Desktop) */
    0x09, 0x04,        /* Usage (Joystick) */
    0xA1, 0x01,        /* Collection (Application) */

    0x09, 0x30,        /* Usage (X) */
    0x16, 0x00, 0x80,  /* Logical Minimum (-32768) */
    0x26, 0xFF, 0x7F,  /* Logical Maximum (+32767) */
    0x75, 0x10,        /* Report Size (16) */
    0x95, 0x01,        /* Report Count (1) */
    0x81, 0x02,        /* Input (Data, Variable, Absolute) */

    0xC0               /* End Collection */
};

/*
 * Send a joystick report over USB HID.
 * value: signed 16-bit axis position [-32768, 32767]
 * Returns USBD_OK, USBD_BUSY, or USBD_FAIL.
 */
uint8_t HID_Joystick_Send(int16_t value)
{
    uint8_t report[2];
    report[0] = (uint8_t)((uint16_t)value & 0xFF);
    report[1] = (uint8_t)((uint16_t)value >> 8);
    return USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}
