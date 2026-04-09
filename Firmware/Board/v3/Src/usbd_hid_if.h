#ifndef __USBD_HID_IF_H
#define __USBD_HID_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_hid.h"
#include "usb_device.h"

/*
 * Send a 16-bit joystick axis value over USB HID.
 * Returns USBD_OK on success, USBD_BUSY if previous transfer is still in
 * progress, or USBD_FAIL if USB is not configured.
 */
uint8_t HID_Joystick_Send(int16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_HID_IF_H */
