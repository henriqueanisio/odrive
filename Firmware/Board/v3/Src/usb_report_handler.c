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

uint8_t HID_GetReport(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    uint16_t wLength   = req->wLength;
    uint8_t reportId   = req->wValue & 0xFF;
    uint8_t reportType = (req->wValue >> 8) & 0xFF;

    if (reportType == HID_REPORT_TYPE_FEATURE)
    {
        switch (reportId)
        {
        case PID_BLOCK_LOAD_REPORT_ID:
        {
            uint8_t buf[5];

            buf[0] = PID_BLOCK_LOAD_REPORT_ID;
            buf[1] = 1; // effectBlockIndex válido (mínimo 1)
            buf[2] = 1; // BLOCK_LOAD_SUCCESS
            buf[3] = 0xFF;
            buf[4] = 0x00;

            uint16_t len = (wLength < 5) ? wLength : 5;

            USBD_CtlSendData(pdev, buf, len);
            return TRUE;
        }

        case PID_POOL_FEATURE_REPORT_ID:
        {
            uint8_t buf[5];

            buf[0] = PID_POOL_FEATURE_REPORT_ID;
            buf[1] = 0x04; // pool size LSB
            buf[2] = 0x00; // pool size MSB
            buf[3] = 4;    // max effects
            buf[4] = 0;

            uint16_t len = (wLength < 5) ? wLength : 5;

            USBD_CtlSendData(pdev, buf, len);
            return TRUE;
        }
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

        uint8_t report_id = pbuf[0];
        ffb_process_report(report_id, &pbuf[1], n - 1);
    }
}
