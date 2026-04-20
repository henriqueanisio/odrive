#include "usbd_def.h"
#include <stdint.h>

/* Counts every FFB Output report received from USB host (games/DirectInput). */
extern volatile uint16_t g_ffb_usb_rx_count;
extern volatile uint8_t  g_ffb_usb_last_rid;

uint8_t HID_GetReport(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

void HID_OutEvent(uint8_t* pbuf, uint16_t n);
