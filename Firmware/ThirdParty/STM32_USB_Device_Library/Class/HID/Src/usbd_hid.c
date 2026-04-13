/**
  ******************************************************************************
  * @file    usbd_hid.c
  * @brief   STM32 USB HID class driver — single interrupt-IN joystick.
  *
  * Config descriptor layout (34 bytes, standalone HID):
  *   Configuration (9) + Interface (9) + HID (9) + Endpoint (7)
  ******************************************************************************
  */

#include "usbd_hid.h"
#include "usbd_ctlreq.h"
#include "ffb_pid.h"
#include "hid_queue.h"

static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);

/* Forward declaration — defined in usbd_hid_if.c (weak) or application */
extern void HID_ODrive_ProcessCommand(const void *cmd);

USBD_ClassTypeDef USBD_HID = {
    USBD_HID_Init,
    USBD_HID_DeInit,
    USBD_HID_Setup,
    NULL,               /* EP0_TxSent */
    USBD_HID_EP0_RxReady, /* EP0_RxReady — handles incoming Feature Reports */
    USBD_HID_DataIn,
    NULL,               /* DataOut */
    NULL,               /* SOF */
    NULL,               /* IsoINIncomplete */
    NULL,               /* IsoOUTIncomplete */
    USBD_HID_GetHSCfgDesc,
    USBD_HID_GetFSCfgDesc,
    USBD_HID_GetOtherSpeedCfgDesc,
    USBD_HID_GetDeviceQualifierDesc,
};

/* FS/HS/OtherSpeed config descriptor — identical for this device */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END = {
    /* ---------- Configuration Descriptor ---------- */
    0x09,                          /* bLength */
    USB_DESC_TYPE_CONFIGURATION,   /* bDescriptorType */
    USB_HID_CONFIG_DESC_SIZ, 0x00, /* wTotalLength */
    0x01,                          /* bNumInterfaces */
    0x01,                          /* bConfigurationValue */
    0x00,                          /* iConfiguration */
    0xC0,                          /* bmAttributes: self-powered */
    0x32,                          /* bMaxPower: 100 mA */

    /* ---------- Interface Descriptor ---------- */
    0x09,                          /* bLength */
    USB_DESC_TYPE_INTERFACE,       /* bDescriptorType */
    0x00,                          /* bInterfaceNumber */
    0x00,                          /* bAlternateSetting */
    0x01,                          /* bNumEndpoints */
    0x03,                          /* bInterfaceClass: HID */
    0x00,                          /* bInterfaceSubClass: no boot */
    0x00,                          /* bInterfaceProtocol: none */
    0x00,                          /* iInterface */

    /* ---------- HID Descriptor ---------- */
    0x09,                          /* bLength */
    HID_DESCRIPTOR_TYPE,           /* bDescriptorType: HID (0x21) */
    0x11, 0x01,                    /* bcdHID: 1.11 */
    0x00,                          /* bCountryCode */
    0x01,                          /* bNumDescriptors */
    HID_REPORT_DESC_TYPE,          /* bDescriptorType: Report (0x22) */
    /* wDescriptorLength — must be 16-bit LE; cannot use the macro directly
     * in a uint8_t array because values > 255 would be truncated silently. */
    (uint8_t)(HID_REPORT_DESC_SIZE & 0xFFU),   /* low  byte = 0xD6 (470) */
    (uint8_t)(HID_REPORT_DESC_SIZE >> 8U),      /* high byte = 0x01 (470) */

    /* ---------- Endpoint Descriptor ---------- */
    0x07,                          /* bLength */
    USB_DESC_TYPE_ENDPOINT,        /* bDescriptorType */
    HID_EPIN_ADDR,                 /* bEndpointAddress: EP1 IN */
    USBD_EP_TYPE_INTR,             /* bmAttributes: Interrupt */
    HID_EPIN_SIZE, 0x00,           /* wMaxPacketSize */
    HID_FS_BINTERVAL,              /* bInterval */
};

/* Device qualifier (required for USB 2.0 compliance, not really used at FS) */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00, 0x02,   /* bcdUSB */
    0x00,         /* bDeviceClass */
    0x00,         /* bDeviceSubClass */
    0x00,         /* bDeviceProtocol */
    0x40,         /* bMaxPacketSize0 */
    0x01,         /* bNumConfigurations */
    0x00,         /* bReserved */
};

static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)cfgidx;

    USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)USBD_malloc(sizeof(USBD_HID_HandleTypeDef));
    if (hhid == NULL) {
        pdev->pClassData = NULL;
        return (uint8_t)USBD_EMEM;
    }

    pdev->pClassData = hhid;

    USBD_LL_OpenEP(pdev, HID_EPIN_ADDR, USBD_EP_TYPE_INTR, HID_EPIN_SIZE);
    pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 1U;

    hhid->state = HID_IDLE;
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)cfgidx;

    USBD_LL_CloseEP(pdev, HID_EPIN_ADDR);
    pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 0U;

    if (pdev->pClassData != NULL) {
        USBD_free(pdev->pClassData);
        pdev->pClassData = NULL;
    }
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
    uint16_t len;
    uint8_t *pbuf;
    uint16_t status_info = 0U;
    USBD_StatusTypeDef ret = USBD_OK;

    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
        switch (req->bRequest) {
        case HID_REQ_SET_PROTOCOL:
            hhid->Protocol = (uint8_t)(req->wValue);
            break;
        case HID_REQ_GET_PROTOCOL:
            USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
            break;
        case HID_REQ_SET_IDLE:
            hhid->IdleState = (uint8_t)(req->wValue >> 8);
            break;
        case HID_REQ_GET_IDLE:
            USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
            break;
        case HID_REQ_SET_REPORT:
            /* Feature Report from host (command or PID FFB report) */
            USBD_CtlPrepareRx(pdev, hhid->FeatureBuf,
                              MIN(req->wLength, HID_FEATURE_REPORT_BUF_SIZE));
            break;
        case HID_REQ_GET_REPORT: {
            /* PID Block Load (0x0F) and PID Pool (0x10) are read by DirectInput */
            static uint8_t pid_resp[8];
            uint8_t report_id = (uint8_t)(req->wValue & 0xFFU);
            uint8_t resp_len  = 0U;
            if (report_id == FFB_REPORT_PID_BLOCK_LOAD) {
                resp_len = ffb_get_block_load_report(pid_resp, sizeof(pid_resp));
            } else if (report_id == FFB_REPORT_PID_POOL) {
                resp_len = ffb_get_pool_report(pid_resp, sizeof(pid_resp));
            }
            if (resp_len > 0U) {
                USBD_CtlSendData(pdev, pid_resp, MIN(resp_len, req->wLength));
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;
        }
        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
            if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_GET_DESCRIPTOR:
            if ((req->wValue >> 8) == HID_REPORT_DESC_TYPE) {
                len = MIN(HID_REPORT_DESC_SIZE, req->wLength);
                pbuf = HID_ReportDesc;
                USBD_CtlSendData(pdev, pbuf, len);
            } else if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE) {
                /* Return just the HID descriptor (9 bytes) from inside CfgDesc */
                pbuf = USBD_HID_CfgDesc + 18U; /* offset past config + interface */
                len = MIN(USB_HID_DESC_SIZ, req->wLength);
                USBD_CtlSendData(pdev, pbuf, len);
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_GET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_SET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                hhid->AltSetting = (uint8_t)(req->wValue);
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_CLEAR_FEATURE:
            break;

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;

    default:
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
        break;
    }

    return (uint8_t)ret;
}

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len)
{
    USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;

    if (pdev->dev_state != USBD_STATE_CONFIGURED) {
        return (uint8_t)USBD_FAIL;
    }
    if (hhid->state == HID_BUSY) {
        return (uint8_t)USBD_BUSY;
    }

    hhid->state = HID_BUSY;
    USBD_LL_Transmit(pdev, HID_EPIN_ADDR, report, len);
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
    /* FeatureBuf[0] = Report ID, FeatureBuf[1..] = payload */
    uint8_t report_id = hhid->FeatureBuf[0];

    if (report_id == 0x03U) {
        /* Vendor command report → existing protocol handler */
        HID_ODrive_ProcessCommand(&hhid->FeatureBuf[1]);
    } else if (report_id >= FFB_REPORT_SET_EFFECT &&
               report_id <= FFB_REPORT_DEVICE_GAIN) {
        /* HID PID FFB reports → force feedback state machine */
        uint16_t payload_len = (uint16_t)(HID_FEATURE_REPORT_BUF_SIZE - 1U);
        ffb_process_report(report_id, &hhid->FeatureBuf[1], payload_len);
    }
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    (void)epnum;
    ((USBD_HID_HandleTypeDef *)pdev->pClassData)->state = HID_IDLE;

    hid_queue_process();
    return (uint8_t)USBD_OK;
}

static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
    *length = sizeof(USBD_HID_CfgDesc);
    return USBD_HID_CfgDesc;
}

static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length)
{
    *length = sizeof(USBD_HID_CfgDesc);
    return USBD_HID_CfgDesc;
}

static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
    *length = sizeof(USBD_HID_CfgDesc);
    return USBD_HID_CfgDesc;
}

static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
    *length = sizeof(USBD_HID_DeviceQualifierDesc);
    return USBD_HID_DeviceQualifierDesc;
}
