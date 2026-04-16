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
static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
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
    USBD_HID_DataOut,    /* DataOut */
    NULL,               /* SOF */
    NULL,               /* IsoINIncomplete */
    NULL,               /* IsoOUTIncomplete */
    USBD_HID_GetHSCfgDesc,
    USBD_HID_GetFSCfgDesc,
    USBD_HID_GetOtherSpeedCfgDesc,
    USBD_HID_GetDeviceQualifierDesc,
};

/* FS/HS/OtherSpeed config descriptor — identical for this device */
/* FS/HS/OtherSpeed config descriptor — identical for this device */
/* usbd_hid.c */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[41] __ALIGN_END = {
    /* Configuration Descriptor (9 bytes) */
    0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x32, // wTotalLength = 41 (0x29)

    /* Interface Descriptor (9 bytes) */
    0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, // bNumEndpoints = 2 (IN e OUT)

    /* HID Descriptor (9 bytes) */
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
    0xDB, 
    0x01,

    /* Endpoint IN Descriptor (7 bytes) - O que envia posição pro PC */
    0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x01,

    /* Endpoint OUT Descriptor (7 bytes) - O que recebe a força do PC */
    0x07, 0x05, 0x01, 0x03, 0x40, 0x00, 0x01, 
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

    USBD_HID_HandleTypeDef *hhid =
        (USBD_HID_HandleTypeDef *)USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

    if (hhid == NULL) {
        pdev->pClassData = NULL;
        return (uint8_t)USBD_EMEM;
    }

    pdev->pClassData = hhid;

    USBD_LL_OpenEP(pdev, 0x81, USBD_EP_TYPE_INTR, 64);

    USBD_LL_OpenEP(pdev, 0x01, USBD_EP_TYPE_INTR, 64);

    hhid->state = HID_IDLE;

    /* preparar OUT */
    USBD_LL_PrepareReceive(
        pdev,
        0x01,
        hhid->OutReportBuf,
        64
    );

    return (uint8_t)USBD_OK;
}

static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)cfgidx;

    USBD_LL_CloseEP(pdev, 0x81);
    USBD_LL_CloseEP(pdev, 0x01);
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
                if ((req->wValue >> 8) == HID_REPORT_DESC_TYPE)
                {
                    USBD_CtlSendData(pdev, HID_ReportDesc,
                        MIN(sizeof(HID_ReportDesc), req->wLength));
                }
                else if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
                {
                    pbuf = USBD_HID_CfgDesc + 18U;

                    len = MIN(USB_HID_DESC_SIZ, req->wLength);
                    USBD_CtlSendData(pdev, pbuf, len);
                }
                else
                {
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
    USBD_LL_Transmit(pdev, 0x81, report, len);
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

static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    // 1. Filtra pelo Endpoint correto (0x01 no seu caso)
    if (epnum == 0x01) 
    {
        USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef*)pdev->pClassData;
        
        // 2. Pegamos o Report ID (byte 0)
        uint8_t report_id = hhid->OutReportBuf[0];

        /* 3. IMPORTANTE: 
           Se ffb_process_report for muito lenta, o joystick (Eixo X) para de responder.
           Garanta que essa função apenas salve os valores em variáveis
           e deixe o cálculo do torque para o loop principal do ODrive. */
        ffb_process_report(report_id, &hhid->OutReportBuf[1], 63);

        // 4. ESSENCIAL: Re-arma o endpoint IMEDIATAMENTE para não perder dados
        // Use o tamanho fixo de 64 bytes para evitar erros de enumeração
        USBD_LL_PrepareReceive(pdev, 0x01, hhid->OutReportBuf, 64);
    }
    
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
