#include "config.h"

#include "usb_reports.h"
#include "usbd_hid.h"
#include "usbd_def.h"
#include "usbd_ioreq.h"

#include "ffb.h"
#include <stdint.h>

#define HID_REPORT_TYPE_INPUT 1
#define HID_REPORT_TYPE_OUTPUT 2
#define HID_REPORT_TYPE_FEATURE 3

#include "usb_report_handler.h"

uint8_t HID_GetReport(USBD_HandleTypeDef *pdev, uint16_t wValue) {
  uint8_t reportId = LOBYTE(wValue);
  uint8_t reportType = HIBYTE(wValue);

  if (reportType == HID_REPORT_TYPE_FEATURE) {
    switch (reportId) {
      case PID_POOL_FEATURE_REPORT_ID:
      {
          uint8_t buf[5];

          buf[0] = PID_POOL_FEATURE_REPORT_ID;

          uint16_t pool = FFB_MAX_EFFECTS;

          buf[1] = pool & 0xFF;
          buf[2] = pool >> 8;
          buf[3] = FFB_MAX_EFFECTS;
          buf[4] = 0;

          USBD_CtlSendData(pdev, buf, 5);
          return TRUE;
      }
      case PID_BLOCK_LOAD_REPORT_ID:
      {
          uint8_t buf[5];

          buf[0] = PID_BLOCK_LOAD_REPORT_ID;

          const PID_BlockLoadReport *data = FFB_GetPidBlockLoad();

          buf[1] = data->effectBlockIndex;
          buf[2] = data->blockLoadStatus;

          uint16_t pool = data->ramPoolAvailable;

          buf[3] = pool & 0xFF;
          buf[4] = pool >> 8;

          USBD_CtlSendData(pdev, buf, 5);
          return TRUE;
      }
    default:
      break;
    }
  }
  return FALSE;
}

/* Counters incremented on every FFB Output report received from USB host (games). */
volatile uint16_t g_ffb_usb_rx_count = 0;
volatile uint8_t  g_ffb_usb_last_rid = 0;

void HID_OutEvent(uint8_t *pbuf, uint16_t n)
{
    if (n >= 1U) {
        g_ffb_usb_rx_count++;
        g_ffb_usb_last_rid = pbuf[0];
    }
    FFB_OnUsbData(pbuf, n);
}
