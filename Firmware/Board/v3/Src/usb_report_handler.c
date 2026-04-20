#include "config.h"

#include "usb_reports.h"
#include "usbd_hid.h"
#include "usbd_def.h"
#include "usbd_ioreq.h"

#include "ffb.h"
#include "ffb_pid.h"
#include <stdint.h>

#define HID_REPORT_TYPE_INPUT 1
#define HID_REPORT_TYPE_OUTPUT 2
#define HID_REPORT_TYPE_FEATURE 3

#include "usb_report_handler.h"

uint8_t HID_GetReport(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    uint16_t wLength   = req->wLength;
    uint8_t reportId   = req->wValue & 0xFF;
    uint8_t reportType = (req->wValue >> 8) & 0xFF;

    if (reportType == HID_REPORT_TYPE_FEATURE)
    {
        uint8_t buf[16];
        uint8_t size = 0;

        switch (reportId)
        {
        case FFB_REPORT_PID_POOL:
            size = ffb_get_pool_report(buf, sizeof(buf));
            break;

        case FFB_REPORT_PID_BLOCK_LOAD:
            size = ffb_get_block_load_report(buf, sizeof(buf));
            break;

        default:
            break;
        }

        if (size > 0)
        {
            uint16_t len = (wLength < size) ? wLength : size;
            USBD_CtlSendData(pdev, buf, len);
            return TRUE;
        }
    }

    /* fallback — nunca quebrar USB */
    uint8_t dummy[1] = {0};
    uint16_t len = (wLength < 1) ? wLength : 1;
    USBD_CtlSendData(pdev, dummy, len);

    return TRUE;
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
