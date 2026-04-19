#include "usbd_hid_if.h"
#include "ffb_pid.h"
#include <string.h>
#include "hid_queue.h"

extern USBD_HandleTypeDef hUsbDeviceFS;


static int8_t HID_Init_FS(void);
static int8_t HID_DeInit_FS(void);
static int8_t HID_OutEvent_FS(uint8_t* pbuf, uint8_t n);

USBD_HID_ItfTypeDef USBD_HID_fops_FS =
{
  HID_ReportDesc_FS,
  HID_Init_FS,
  HID_DeInit_FS,
  HID_OutEvent_FS
};

static int8_t HID_Init_FS(void) {
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

static int8_t HID_DeInit_FS(void) {
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

static int8_t HID_OutEvent_FS(uint8_t* pbuf, uint8_t n)
{
  /* USER CODE BEGIN 6 */
  HID_OutEvent(pbuf, n);
  return (USBD_OK);
  /* USER CODE END 6 */
}


/* ── HID_Joystick_Send ───────────────────────────────────────────────────── */
uint8_t HID_Joystick_Send(int16_t value)
{
    uint8_t report[1 + HID_JOYSTICK_PAYLOAD_SIZE];
    report[0] = HID_REPORT_ID_JOYSTICK;
    report[1] = 0;                          /* buttons byte — all released    */
    report[2] = (uint8_t)(value & 0xFF);    /* X axis low byte                */
    report[3] = (uint8_t)(value >> 8);      /* X axis high byte               */
    return hid_queue_push(report, sizeof(report));
}

/* ── HID_ODrive_SendTelemetry ────────────────────────────────────────────── */
uint8_t HID_ODrive_SendTelemetry(const HID_TelemetryPayload_t *payload)
{
    uint8_t report[1 + HID_TELEMETRY_PAYLOAD_SIZE];
    report[0] = HID_REPORT_ID_TELEMETRY;
    memcpy(&report[1], payload, HID_TELEMETRY_PAYLOAD_SIZE);
    return hid_queue_push(report, sizeof(report));
}

/* ── HID_ODrive_SendConfigResponse ──────────────────────────────────────── */
uint8_t HID_ODrive_SendConfigResponse(uint16_t param_id, float value)
{
    uint8_t report[1 + 2 + 4];
    report[0] = HID_REPORT_ID_CONFIG_RESP;
    memcpy(&report[1], &param_id, 2);
    memcpy(&report[3], &value,    4);
    bool ok = hid_queue_push(report, sizeof(report));
    hid_queue_process();
    return ok ? (uint8_t)USBD_OK : (uint8_t)USBD_BUSY;
}

/* ── HID_ODrive_ProcessCommand (weak default) ────────────────────────────── */
__attribute__((weak)) void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *cmd)
{
    (void)cmd;
}
