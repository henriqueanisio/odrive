#include "usbd_hid_if.h"
#include "ffb_pid.h"
#include <string.h>
#include "hid_queue.h"

__ALIGN_BEGIN uint8_t HID_ReportDesc[] __ALIGN_END = {

0x05, 0x0F,
0x09, 0x01,
0xA1, 0x01,

// PID STATE
0x85, 0x02,
0x09, 0x92,
0xA1, 0x02,
0x09, 0x9F,
0x15, 0x00,
0x25, 0x01,
0x75, 0x01,
0x95, 0x01,
0x81, 0x02,
0x75, 0x01,
0x95, 0x07,
0x81, 0x03,
0xC0,

// CREATE NEW EFFECT (CRÍTICO)
0x85, 0x12,
0x09, 0xAB,
0xA1, 0x02,
0x09, 0xAC,
0x15, 0x00,
0x26, 0xFF, 0x00,
0x75, 0x08,
0x95, 0x04,
0xB1, 0x02,
0xC0,

// PID POOL (CRÍTICO)
0x85, 0x11,
0x09, 0x7F,
0xA1, 0x02,
0x09, 0x80,
0x15, 0x00,
0x26, 0xFF, 0x00,
0x75, 0x08,
0x95, 0x08,
0xB1, 0x02,
0xC0,

// SET EFFECT
0x85, 0x01,
0x09, 0x21,
0xA1, 0x02,
0x09, 0x22,
0x15, 0x01,
0x25, 0x10,
0x75, 0x08,
0x95, 0x01,
0x91, 0x02,
0xC0,

// EFFECT OPERATION
0x85, 0x0A,
0x09, 0x77,
0xA1, 0x02,
0x09, 0x22,
0x15, 0x01,
0x25, 0x10,
0x75, 0x08,
0x95, 0x01,
0x91, 0x02,
0xC0,

0xC0,
};

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
