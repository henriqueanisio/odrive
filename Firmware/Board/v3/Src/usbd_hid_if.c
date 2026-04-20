#include "usbd_hid_if.h"
#include "ffb_pid.h"
#include <string.h>
#include "hid_queue.h"
#include "ffb_pid.h"
#include "usb_report_handler.h"

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
  if (n < 1U) return (USBD_OK);

  if (pbuf[0] == HID_REPORT_ID_COMMAND) {
    /* Vendor command report 0x21 — route to protocol handler.
     * pbuf[0] = report_id, pbuf[1..] = HID_CommandPayload_t fields. */
    if (n >= (uint8_t)(sizeof(HID_CommandPayload_t) + 1U)) {
      HID_ODrive_ProcessCommand((const HID_CommandPayload_t *)&pbuf[1]);
    }
  } else {
    /* FFB output reports (0x01–0x0D, 0x07 CreateNewEffect) */
    HID_OutEvent(pbuf, n);
  }

  return (USBD_OK);
}

static int8_t HID_OutEvent_FS(uint8_t* pbuf, uint8_t n)
{
  if (n < 1U) return (USBD_OK);

  uint8_t report_id = pbuf[0];

  // 1. Mantém o tratamento do seu comando Vendor (ODrive config)
  if (report_id == HID_REPORT_ID_COMMAND) {
   /* Vendor command report 0x21 — route to protocol handler.
     * pbuf[0] = report_id, pbuf[1..] = HID_CommandPayload_t fields. */
    if (n >= (uint8_t)(sizeof(HID_CommandPayload_t) + 1U)) {
      HID_ODrive_ProcessCommand((const HID_CommandPayload_t *)&pbuf[1]);
    }
  } 
  // 2. Encaminha os Reports de FFB (IDs entre 1 e 0x1F conforme seu descritor)
  // O ponteiro de dados começa em pbuf[1] e o tamanho útil é n-1
  else if (report_id > 0 && report_id < 0x20) {
    ffb_pid_process_report(report_id, &pbuf[1], n - 1);
  }

  return (USBD_OK);
}


/* ── HID_Joystick_Send ───────────────────────────────────────────────────── */
/* Descriptor: steering(int16) + accelerator(int16) + brake(int16) + buttons(uint32)
 * Only steering is used; accelerator/brake sent as 0 (neutral). */
uint8_t HID_Joystick_Send(int16_t steering)
{
    uint8_t report[1 + HID_JOYSTICK_PAYLOAD_SIZE];
    report[0]  = HID_REPORT_ID_JOYSTICK;
    report[1]  = (uint8_t)((uint16_t)steering & 0xFFU);
    report[2]  = (uint8_t)((uint16_t)steering >> 8U);
    report[3]  = 0x00U;  /* accelerator low  — neutral = 0 */
    report[4]  = 0x00U;  /* accelerator high               */
    report[5]  = 0x00U;  /* brake low        — neutral = 0 */
    report[6]  = 0x00U;  /* brake high                     */
    report[7]  = 0x00U;  /* buttons [7:0]                  */
    report[8]  = 0x00U;  /* buttons [15:8]                 */
    report[9]  = 0x00U;  /* buttons [23:16]                */
    report[10] = 0x00U;  /* buttons [31:24]                */
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
