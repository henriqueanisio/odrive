#include "usbd_def.h"
#include <stdint.h>

uint8_t HID_GetReport(USBD_HandleTypeDef *pdev, uint16_t wValue);

void HID_OutEvent(uint8_t* pbuf, uint8_t n);
