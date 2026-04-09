/**
 * hid_app_integration.cpp
 *
 * Provides the strong definition of HID_ODrive_ProcessCommand() and wires
 * the protocol module to the ODrive C++ axis API.
 *
 * HOW TO USE
 * ──────────
 * 1. Add this file to your build system.
 * 2. Call hid_app_init() from your start_odrive_tasks() or equivalent
 *    startup function, AFTER USB is initialised.
 * 3. Call protocol_process_pending() once per main-loop iteration or from
 *    a dedicated low-priority RTOS task.
 *
 * EXAMPLE (odrive_main.cpp):
 *
 *   extern void hid_app_init(void);
 *
 *   void start_odrive_tasks() {
 *       // ... existing init ...
 *       hid_app_init();
 *   }
 *
 *   // In your 10 ms telemetry task:
 *   void telemetry_task(void *arg) {
 *       for (;;) {
 *           hid_send_telemetry();
 *           protocol_process_pending();
 *           osDelay(10);
 *       }
 *   }
 */

#include "protocol.h"
#include "flash_storage.h"
#include "usbd_hid_if.h"
#include <MotorControl/odrive_main.h>   /* odrv, Axis */
#include <string.h>

/* ── Convenience accessors ─────────────────────────────────────────────────── */
static inline Axis& axis0() { return odrv.get_axis(0); }

/* ── Application callbacks ─────────────────────────────────────────────────── */
static void app_set_axis_state(uint8_t state)
{
    axis0().requested_state_ = static_cast<Axis::AxisState>(state);
}

static void app_set_input_pos(float pos)
{
    axis0().controller_.input_pos_ = pos;
}

static void app_clear_errors(void)
{
    axis0().error_  = Axis::ERROR_NONE;
    axis0().motor_.error_   = Motor::ERROR_NONE;
    axis0().encoder_.error_ = Encoder::ERROR_NONE;
}

static void app_enter_dfu(void)
{
    odrv.enter_dfu_mode();
}

/* ── HID_ODrive_ProcessCommand — strong override ─────────────────────────────
 * Called from usbd_hid.c :: USBD_HID_EP0_RxReady (USB interrupt context).
 * Must not block.
 * ─────────────────────────────────────────────────────────────────────────── */
extern "C" void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *raw)
{
    /* The raw pointer points to FeatureBuf[1] — same memory layout as
     * HID_Command_t, so a cast is safe (both packed, same byte order).       */
    protocol_dispatch(reinterpret_cast<const HID_Command_t *>(raw));
}

/* ── hid_send_telemetry — call from telemetry task ──────────────────────────
 * Fills HID_TelemetryPayload_t from live ODrive state and sends it.         */
extern "C" void hid_send_telemetry(void)
{
    HID_TelemetryPayload_t t;
    memset(&t, 0, sizeof(t));

    Axis &ax = axis0();

    t.pos_estimate    = ax.encoder_.pos_estimate_;
    t.vel_estimate    = ax.encoder_.vel_estimate_;
    t.vbus_voltage    = odrv.vbus_voltage_;
    t.current_lim     = ax.motor_.config_.current_lim;
    t.input_pos       = ax.controller_.input_pos_;
    t.Iq_measured     = ax.motor_.current_control_.Iq_measured_;
    t.phase_resistance = ax.motor_.config_.phase_resistance;
    t.phase_inductance = ax.motor_.config_.phase_inductance;

    t.current_state  = static_cast<uint8_t>(ax.current_state_);
    t.flags          = (ax.encoder_.is_ready_        ? 0x01U : 0U)
                     | (ax.motor_.is_calibrated_      ? 0x02U : 0U);
    t.axis_error     = static_cast<uint8_t>(ax.error_    != 0);
    t.motor_error    = static_cast<uint8_t>(ax.motor_.error_   != 0);
    t.encoder_error  = static_cast<uint8_t>(ax.encoder_.error_ != 0);

    /* Also send joystick report (keeps joy.cpl working) */
    int16_t joy_x = static_cast<int16_t>(
        ax.encoder_.pos_estimate_ * (32767.0f / 100.0f));   /* scale as needed */
    HID_Joystick_Send(joy_x);

    HID_ODrive_SendTelemetry(&t);
}

/* ── hid_app_init ───────────────────────────────────────────────────────────── */
extern "C" void hid_app_init(void)
{
    /* Try to restore saved config; fall back to compile-time defaults */
    if (!flash_load_config()) {
        config_init();   /* loads factory defaults */
    }

    /* Wire callbacks */
    static const ProtocolCallbacks_t cb = {
        .set_axis_state = app_set_axis_state,
        .set_input_pos  = app_set_input_pos,
        .clear_errors   = app_clear_errors,
        .enter_dfu      = app_enter_dfu,
    };
    protocol_init(&cb);
}
