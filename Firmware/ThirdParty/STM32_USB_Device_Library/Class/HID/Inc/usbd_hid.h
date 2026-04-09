/**
  ******************************************************************************
  * @file    usbd_hid.h
  * @brief   Header file for the usbd_hid.c HID class driver.
  ******************************************************************************
  */

#ifndef __USB_HID_H
#define __USB_HID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"

/** @defgroup USBD_HID_Exported_Defines */

#define HID_EPIN_ADDR               0x81U
#define HID_EPIN_SIZE               0x04U
#define HID_FS_BINTERVAL            0x0AU   /* 10 ms */

#define USB_HID_CONFIG_DESC_SIZ     34U
#define USB_HID_DESC_SIZ            9U

/* Size of the joystick report descriptor defined in usbd_hid_if.c */
#define HID_REPORT_DESC_SIZE        21U

#define HID_DESCRIPTOR_TYPE         0x21U
#define HID_REPORT_DESC_TYPE        0x22U

/* HID class-specific requests */
#define HID_REQ_SET_PROTOCOL        0x0BU
#define HID_REQ_GET_PROTOCOL        0x03U
#define HID_REQ_SET_IDLE            0x0AU
#define HID_REQ_GET_IDLE            0x02U
#define HID_REQ_SET_REPORT          0x09U
#define HID_REQ_GET_REPORT          0x01U

/* TX state for preventing overlapping transfers */
#define HID_IDLE                    0U
#define HID_BUSY                    1U

typedef struct {
    uint32_t Protocol;
    uint32_t IdleState;
    uint32_t AltSetting;
    volatile uint32_t state;
} USBD_HID_HandleTypeDef;

/** Report descriptor is defined in usbd_hid_if.c */
extern uint8_t HID_ReportDesc[HID_REPORT_DESC_SIZE];

extern USBD_ClassTypeDef USBD_HID;

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __USB_HID_H */
