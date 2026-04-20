/* ffb_compat.c — Bridges the legacy ffb_pid.h API used by hid_app_integration.cpp
 * to the new modular FFB engine (ffb_engine/ffb_forces/ffb_axis/ffb_report_handler).
 *
 * Position scale: 1 turn → 32767 units (matches HID axis ±32767 range).
 * Force scale:    FFBEngine_CalculateForce() returns ±16383;
 *                 scale = ffb_max_torque × ffb_gain / 16383.
 */

#include "ffb_pid.h"
#include "ffb.h"
#include "ffb_engine.h"
#include "ffb_axis.h"
#include "usb_reports.h"
#include "usb_report_handler.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* Globals defined in ffb_engine.c */
extern volatile PIDStateReport g_state;
extern volatile uint8_t        g_deviceGain;

/* Diagnostic counters: reads from usb_report_handler globals (real USB traffic). */

/* ── ffb_init ────────────────────────────────────────────────────────────── */
void ffb_init(void)
{
    FreeAllEffects();
    g_state.actuatorsEnabled   = FALSE;
    g_state.devicePaused       = 0;
    g_state.effectPlaying      = 0;
    g_state.effectBlockIndex   = 0;
    g_deviceGain               = 255;
    g_ffb_usb_rx_count         = 0;
    g_ffb_usb_last_rid         = 0;
}

/* ── ffb_process_report ──────────────────────────────────────────────────── */
/* hid_app_integration passes (report_id, data_after_id, len).
 * FFB_OnUsbData expects a contiguous buffer [report_id, payload...].      */
void ffb_process_report(uint8_t report_id, const uint8_t *data, uint16_t len)
{
    uint8_t buf[64];
    if (len + 1U > sizeof(buf)) return;

    buf[0] = report_id;
    if (len > 0U && data != NULL) {
        memcpy(&buf[1], data, len);
    }

    /* Route through HID_OutEvent so global counters are updated */
    HID_OutEvent(buf, (uint16_t)(len + 1U));
}

/* ── ffb_actuators_enabled ───────────────────────────────────────────────── */
bool ffb_actuators_enabled(void)
{
    return (bool)g_state.actuatorsEnabled;
}

/* ── ffb_compute_torque ──────────────────────────────────────────────────── */
float ffb_compute_torque(float pos_turns, float vel_turns_s)
{
    (void)vel_turns_s; /* velocity is derived internally by FFB_Axis_Update */

    /* Convert turns → normalized ±32767 units */
    int32_t pos_norm = (int32_t)(pos_turns * 32767.0f);
    FFB_Axis_Update(pos_norm);

    if (!g_state.actuatorsEnabled || g_state.devicePaused) {
        return 0.0f;
    }

    int16_t raw_force = FFBEngine_CalculateForce();

    /* Scale to Nm using user-configured gain and max torque */
    float scale = g_config.ffb_max_torque * g_config.ffb_gain / 16383.0f;
    float torque = (float)raw_force * scale;

    /* Clamp to configured peak torque */
    if (torque >  g_config.ffb_max_torque) torque =  g_config.ffb_max_torque;
    if (torque < -g_config.ffb_max_torque) torque = -g_config.ffb_max_torque;

    return torque;
}

/* ── ffb_get_block_load_report ───────────────────────────────────────────── */
uint8_t ffb_get_block_load_report(uint8_t *buf, uint8_t buf_size)
{
    if (buf_size < sizeof(PID_BlockLoadReport)) return 0U;
    const volatile PID_BlockLoadReport *src = FFB_GetPidBlockLoad();
    memcpy(buf, (const void *)src, sizeof(PID_BlockLoadReport));
    return (uint8_t)sizeof(PID_BlockLoadReport);
}

/* ── ffb_get_pool_report ─────────────────────────────────────────────────── */
uint8_t ffb_get_pool_report(uint8_t *buf, uint8_t buf_size)
{
    if (buf_size < sizeof(PID_PoolFeatureReport)) return 0U;
    PID_PoolFeatureReport report;
    PIDPoolFeatureReport_Init(&report);
    memcpy(buf, &report, sizeof(PID_PoolFeatureReport));
    return (uint8_t)sizeof(PID_PoolFeatureReport);
}

/* ── ffb_get_diag_stats ──────────────────────────────────────────────────── */
FFB_DiagStats_t ffb_get_diag_stats(void)
{
    FFB_DiagStats_t d;
    d.rx_count = g_ffb_usb_rx_count;
    d.last_rid = g_ffb_usb_last_rid;
    d.actv     = (uint8_t)((g_state.actuatorsEnabled ? 0x01U : 0x00U) |
                            (g_state.effectPlaying    ? 0x02U : 0x00U));
    return d;
}
