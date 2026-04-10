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

/* ── Convenience accessor ───────────────────────────────────────────────────── */
static inline Axis& axis0() { return odrv.get_axis(0); }

/* ── Forward declaration ────────────────────────────────────────────────────── */
static void app_apply_config(void);

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
    axis0().error_           = Axis::ERROR_NONE;
    axis0().motor_.error_    = Motor::ERROR_NONE;
    axis0().encoder_.error_  = Encoder::ERROR_NONE;
}

static void app_enter_dfu(void)
{
    odrv.enter_dfu_mode();
}

/* ── app_apply_config ────────────────────────────────────────────────────────
 * Copies all g_config values to the live ODrive objects.
 * Called at startup (after flash load) and on CALL_APPLY_CONFIG.
 * ─────────────────────────────────────────────────────────────────────────── */
static void app_apply_config(void)
{
    Axis &ax = axis0();

    /* ── Motor ── */
    ax.motor_.config_.current_lim                   = g_config.current_lim;
    ax.motor_.config_.torque_constant               = g_config.torque_constant;
    ax.motor_.config_.pole_pairs                    = g_config.pole_pairs;
    ax.motor_.config_.motor_type                    = static_cast<Motor::MotorType>(g_config.motor_type);
    ax.motor_.config_.current_control_bandwidth     = g_config.current_control_bandwidth;
    ax.motor_.config_.calibration_current           = g_config.calibration_current;
    ax.motor_.config_.resistance_calib_max_voltage  = g_config.resistance_calib_max_voltage;
    ax.motor_.config_.pre_calibrated                = g_config.motor_pre_calibrated != 0;

    /* ── Encoder ── */
    ax.encoder_.config_.mode                        = static_cast<Encoder::Mode>(g_config.encoder_mode);
    ax.encoder_.config_.cpr                         = g_config.encoder_cpr;
    ax.encoder_.config_.bandwidth                   = g_config.encoder_bandwidth;
    ax.encoder_.config_.abs_spi_cs_gpio_pin         = static_cast<uint8_t>(g_config.abs_spi_cs_gpio_pin);
    ax.encoder_.config_.pre_calibrated              = g_config.encoder_pre_calibrated != 0;
    /* encoder_direction & offset are applied after calibration, not forced here */

    /* ── Controller ── */
    ax.controller_.config_.control_mode             = static_cast<Controller::ControlMode>(g_config.control_mode);
    ax.controller_.config_.vel_limit                = g_config.vel_limit;
    ax.controller_.config_.pos_gain                 = g_config.pos_gain;
    ax.controller_.config_.vel_gain                 = g_config.vel_gain;
    ax.controller_.config_.vel_integrator_gain      = g_config.vel_integrator_gain;

    /* ── DC Bus / Brake ── */
    odrv.config_.dc_bus_undervoltage_trip_level     = g_config.vbus_undervoltage;
    odrv.config_.dc_bus_overvoltage_trip_level      = g_config.vbus_overvoltage;
    odrv.config_.enable_brake_resistor              = g_config.enable_brake_resistor != 0;
    odrv.config_.brake_resistance                   = g_config.brake_resistance;
}

/* ── HID_ODrive_ProcessCommand — strong override ─────────────────────────────
 * Called from USB ISR (via USBD_HID_EP0_RxReady). Must not block.
 * ─────────────────────────────────────────────────────────────────────────── */
extern "C" void HID_ODrive_ProcessCommand(const HID_CommandPayload_t *raw)
{
    protocol_dispatch(reinterpret_cast<const HID_Command_t *>(raw));
}

/* ── hid_send_telemetry — call every ~10 ms from telemetry task ─────────────── */
extern "C" void hid_send_telemetry(void)
{
    HID_TelemetryPayload_t t;
    memset(&t, 0, sizeof(t));

    Axis &ax = axis0();

    t.pos_estimate     = ax.encoder_.pos_estimate_.present().value_or(0.0f);
    t.vel_estimate     = ax.encoder_.vel_estimate_.present().value_or(0.0f);
    t.vbus_voltage     = odrv.vbus_voltage_;
    t.current_lim      = ax.motor_.config_.current_lim;
    t.input_pos        = ax.controller_.input_pos_;
    t.Iq_measured      = ax.motor_.current_control_.Iq_measured_;
    t.phase_resistance = ax.motor_.config_.phase_resistance;
    t.phase_inductance = ax.motor_.config_.phase_inductance;

    t.current_state    = static_cast<uint8_t>(ax.current_state_);
    t.flags            = (ax.encoder_.is_ready_        ? 0x01U : 0U)
                       | (ax.motor_.is_calibrated_      ? 0x02U : 0U);
    t.axis_error       = static_cast<uint8_t>(ax.error_          != 0);
    t.motor_error      = static_cast<uint8_t>(ax.motor_.error_   != 0);
    t.encoder_error    = static_cast<uint8_t>(ax.encoder_.error_ != 0);

    /* Alternate joystick and telemetry sends to avoid HID_BUSY on the shared
       interrupt IN endpoint. Telemetry every call (~10 ms); joystick every
       10th call (~100 ms), well after the previous packet has been ACK'd. */
    static uint8_t joy_divider = 0U;
    if (++joy_divider >= 10U) {
        joy_divider = 0U;
        int16_t joy_x = static_cast<int16_t>(
            ax.encoder_.pos_estimate_.present().value_or(0.0f) * (32767.0f / 100.0f));
        HID_Joystick_Send(joy_x);
        /* yield so the joystick packet can be transmitted before telemetry */
        osDelay(1);
    }

    HID_ODrive_SendTelemetry(&t);
}

/* ── hid_app_init ────────────────────────────────────────────────────────────── */
extern "C" void hid_app_init(void)
{
    /* Load config from flash; fall back to factory defaults */
    if (!flash_load_config()) {
        config_init();
    }

    /* Apply loaded config to ODrive live objects immediately */
    app_apply_config();

    static const ProtocolCallbacks_t cb = {
        .set_axis_state = app_set_axis_state,
        .set_input_pos  = app_set_input_pos,
        .clear_errors   = app_clear_errors,
        .enter_dfu      = app_enter_dfu,
        .apply_config   = app_apply_config,
    };
    protocol_init(&cb);
}
