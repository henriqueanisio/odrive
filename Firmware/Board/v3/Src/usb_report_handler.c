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

if (reportType == HID_REPORT_TYPE_FEATURE) {
    switch (reportId) {
      case PID_POOL_FEATURE_REPORT_ID: {
        PID_PoolFeatureReport report;
        PIDPoolFeatureReport_Init(&report);
        USBD_CtlSendData(pdev, (uint8_t *)&report, sizeof(PID_PoolFeatureReport));
        return TRUE;
      }
      
      case PID_BLOCK_LOAD_REPORT_ID: {
        PID_BlockLoadReport data = *FFB_GetPidBlockLoad();
        USBD_CtlSendData(pdev, (uint8_t *)&data, sizeof(PID_BlockLoadReport));
        return TRUE;
      }
    default:
      break;
    }
  }


/* Counters incremented on every FFB Output report received from USB host (games). */
volatile uint16_t g_ffb_usb_rx_count = 0;
volatile uint8_t  g_ffb_usb_last_rid = 0;

void HID_OutEvent(uint8_t *pbuf, uint16_t n)
{
    if (n >= 1U) {
        g_ffb_usb_rx_count++;
        g_ffb_usb_last_rid = pbuf[0];

        uint8_t report_id = pbuf[0];
        ffb_process_report(report_id, &pbuf[1], n - 1);
    }
}
