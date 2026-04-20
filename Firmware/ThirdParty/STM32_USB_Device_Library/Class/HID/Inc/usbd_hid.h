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

#define HID_EPIN_ADDR                 0x81U
#define HID_EPIN_SIZE                 0x40U

#define HID_EPOUT_ADDR                0x01U
#define HID_EPOUT_SIZE                0x40U

#define USB_HID_CONFIG_DESC_SIZ       41U
#define USB_HID_DESC_SIZ              9U

#ifndef HID_HS_BINTERVAL
#define HID_HS_BINTERVAL            0x05U
#endif /* HID_HS_BINTERVAL */

#ifndef HID_FS_BINTERVAL
#define HID_FS_BINTERVAL            0x05U
#endif /* HID_FS_BINTERVAL */

#ifndef USBD_HID_OUTREPORT_BUF_SIZE
#define USBD_HID_OUTREPORT_BUF_SIZE  0x40U
#endif /* USBD_HID_OUTREPORT_BUF_SIZE */
#ifndef USBD_HID_REPORT_DESC_SIZE
#define USBD_HID_REPORT_DESC_SIZE   163U
#endif /* USBD_HID_REPORT_DESC_SIZE */

#define HID_DESCRIPTOR_TYPE           0x21U
#define HID_REPORT_DESC               0x22U

#define HID_REQ_SET_PROTOCOL          0x0BU
#define HID_REQ_GET_PROTOCOL          0x03U

#define HID_REQ_SET_IDLE              0x0AU
#define HID_REQ_GET_IDLE              0x02U

#define HID_REQ_SET_REPORT            0x09U
#define HID_REQ_GET_REPORT            0x01U

typedef enum
{
  HID_IDLE = 0U,
  HID_BUSY,
}
HID_StateTypeDef;

typedef struct _USBD_HID_Itf
{
  uint8_t                  *pReport;
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* OutEvent)(uint8_t* buf, uint8_t n);

} USBD_HID_ItfTypeDef;

typedef struct
{
  uint8_t              Report_buf[USBD_HID_OUTREPORT_BUF_SIZE];
  uint32_t             Protocol;
  uint32_t             IdleState;
  uint32_t             AltSetting;
  uint32_t             IsReportAvailable;
  HID_StateTypeDef     state;
}
USBD_HID_HandleTypeDef;

extern USBD_ClassTypeDef  USBD_HID;

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);

uint8_t  USBD_HID_RegisterInterface(USBD_HandleTypeDef   *pdev,
                                           USBD_HID_ItfTypeDef *fops);

#ifdef __cplusplus
}
#endif

#endif /* __USB_HID_H */
