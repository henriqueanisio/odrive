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
#include "ffb_pid.h"
#include "usb_device.h"          /* hUsbDeviceFS — for USB state check        */
#include <MotorControl/odrive_main.h>   /* odrv, Axis */
#include <MotorControl/controller.hpp>  /* Controller::ControlMode, InputMode  */
#include <string.h>

/* ── Convenience accessor ───────────────────────────────────────────────────── */
static inline Axis& axis0() { return odrv.get_axis(0); }

/* ── Home / zero-position offset (set via CALL_SET_HOME) ────────────────────── */
static float s_home_offset = 0.0f;

/* ── Forward declaration ────────────────────────────────────────────────────── */
static void app_apply_config(void);

/* ── Application callbacks ─────────────────────────────────────────────────── */
static void app_set_axis_state(uint8_t state)
{
    /* clear_errors() also re-arms the brake resistor when enable_brake_resistor=true */
    odrv.clear_errors();
    axis0().requested_state_ = static_cast<Axis::AxisState>(state);
}

static void app_set_input_pos(float pos)
{
    /* Clamp to ±half of steering_max_lock in turns (e.g. 900° → ±1.25 turns) */
    float half_turns = g_config.steering_max_lock / 720.0f;
    if (pos >  half_turns) pos =  half_turns;
    if (pos < -half_turns) pos = -half_turns;
    /* pos from GUI is relative to home; add offset so controller sees absolute position */
    axis0().controller_.input_pos_ = pos + s_home_offset;
}

static void app_save_config(void)
{
    /* STM32F405 flash sector erase disables all interrupts for ~400-800 ms.
     * The motor control timer (TIM1/TIM8) fires during this window and cannot
     * be served, causing TIMER_UPDATE_MISSED / MOTOR_FAILED on the next cycle.
     * Clearing errors after the erase completes makes the fault transparent
     * to the user — the motor resumes normally once the ISR can run again. */
    flash_save_config();
    odrv.clear_errors();
}

static void app_set_home(void)
{
    /* Capture current encoder position as the new zero reference.
     * All subsequent pos_estimate values in telemetry are reported
     * relative to this point.  input_pos commands from the GUI are
     * also relative, so we add s_home_offset inside app_set_input_pos. */
    s_home_offset = axis0().encoder_.pos_estimate_.present().value_or(0.0f);
    /* Keep the controller at the new home (pos = 0 relative to home) */
    axis0().controller_.input_pos_ = s_home_offset;
}

static void app_clear_errors(void)
{
    /* Delegates to ODrive::clear_errors() so the brake resistor gets re-armed */
    odrv.clear_errors();
}

static void app_enter_dfu(void)
{
    odrv.enter_dfu_mode();
}

/* ── app_apply_config_core ───────────────────────────────────────────────────
 * Copies all g_config values to the live ODrive objects.
 * Does NOT check for encoder mode changes — safe to call at startup.
 * ─────────────────────────────────────────────────────────────────────────── */
static void app_apply_config_core(void)
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
    /* Restore measured calibration values so the current controller is correct
     * even when skipping motor calibration (pre_calibrated = true). */
    if (g_config.phase_resistance > 0.0f) {
        ax.motor_.config_.phase_resistance = g_config.phase_resistance;
    }
    if (g_config.phase_inductance > 0.0f) {
        ax.motor_.config_.phase_inductance = g_config.phase_inductance;
    }
    /* Motor::apply_config() sets is_calibrated_ = config_.pre_calibrated at boot,
     * but we call app_apply_config_core() at runtime too.  Sync is_calibrated_
     * here so the encoder-offset-calibration state machine gate (!is_calibrated_)
     * works correctly without requiring a reboot after the user sets motor
     * pre_calibrated = true.  Only raise the flag — never clear it at runtime
     * so an in-session motor calibration result is not accidentally discarded. */
    if (g_config.motor_pre_calibrated != 0) {
        ax.motor_.is_calibrated_ = true;
    }

    /* ── Encoder ── */
    ax.encoder_.config_.mode                        = static_cast<Encoder::Mode>(g_config.encoder_mode);
    ax.encoder_.config_.cpr                         = g_config.encoder_cpr;
    ax.encoder_.config_.bandwidth                   = g_config.encoder_bandwidth;
    ax.encoder_.config_.abs_spi_cs_gpio_pin         = static_cast<uint8_t>(g_config.abs_spi_cs_gpio_pin);
    ax.encoder_.config_.pre_calibrated              = g_config.encoder_pre_calibrated != 0;
    ax.encoder_.config_.use_index                   = g_config.encoder_use_index != 0;
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

    /* Re-run SPI setup so CLK polarity / CS pin / mode_ take effect.
     * For incremental encoders this is a no-op (timer was initialised at boot). */
    ax.encoder_.setup();

    if (g_config.encoder_mode != 0) return;

    TIM_HandleTypeDef *htim = ax.encoder_.timer_;

    // 🔥 só garante que está rodando
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

/* ── app_apply_config ────────────────────────────────────────────────────────
 * User-triggered apply (via CALL_APPLY_CONFIG from GUI).
 * Applies config AND detects encoder mode changes that require a reboot.
 *
 * WHY TWO FUNCTIONS:
 *   Encoder mode changes require the STM32 hardware timer to be reconfigured,
 *   which only happens cleanly at boot.  We detect a mode change here and
 *   issue a soft-reset so the timer is properly re-initialised.
 *
 *   This check MUST NOT run at startup (hid_app_init uses app_apply_config_core
 *   instead) because at startup the ODrive live mode is always the default
 *   (INCREMENTAL = 0) before any config is applied, which would falsely trigger
 *   a reboot every time the board powers on with a non-incremental mode saved.
 * ─────────────────────────────────────────────────────────────────────────── */
static void app_apply_config(void)
{
    Axis &ax = axis0();

    /* Snapshot current live mode BEFORE applying so we can detect a change. */
    int32_t live_mode = static_cast<int32_t>(ax.encoder_.config_.mode);

    app_apply_config_core();

    if (live_mode != g_config.encoder_mode) {
        /* Mode changed: persist to flash then reboot so the timer is
         * re-initialised correctly for the new mode.
         * clear_errors() after the flash erase suppresses TIMER_UPDATE_MISSED. */
        flash_save_config();
        odrv.clear_errors();
        HAL_Delay(20);       /* allow USB ACK to reach host before reset */
        NVIC_SystemReset();
        /* never reached */
    }
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

    float pos_abs      = ax.encoder_.pos_estimate_.present().value_or(0.0f);
    t.pos_estimate     = pos_abs - s_home_offset;
    t.vel_estimate     = ax.encoder_.vel_estimate_.present().value_or(0.0f);
    t.vbus_voltage     = odrv.vbus_voltage_;
    t.current_lim      = ax.motor_.config_.current_lim;
    t.input_pos        = ax.controller_.input_pos_ - s_home_offset;
    t.Iq_measured      = ax.motor_.current_control_.Iq_measured_;
    t.phase_resistance = ax.motor_.config_.phase_resistance;
    t.phase_inductance = ax.motor_.config_.phase_inductance;

    t.current_state    = static_cast<uint8_t>(ax.current_state_);
    t.flags            = (ax.encoder_.is_ready_ ? 0x01U : 0U)
                       | (ax.motor_.is_calibrated_ ? 0x02U : 0U);
    t.axis_error       = static_cast<uint32_t>(ax.error_);
    t.motor_error      = static_cast<uint32_t>(ax.motor_.error_);
    t.encoder_error    = static_cast<uint32_t>(ax.encoder_.error_);
    t.mag_agc          = ax.encoder_.abs_agc_;
    t.mag_flags        = ax.encoder_.abs_diag_flags_;

    /* ───────────── JOYSTICK ───────────── */

    float half_turns = g_config.steering_max_lock / 720.0f;

    float joy_f = (t.pos_estimate / half_turns) * 32767.0f;

    if (joy_f >  32767.0f) joy_f =  32767.0f;
    if (joy_f < -32767.0f) joy_f = -32767.0f;

    int16_t joy_x = static_cast<int16_t>(joy_f);

    HID_Joystick_Send(joy_x);

    /* ───────────── TELEMETRIA ───────────── */

    HID_ODrive_SendTelemetry(&t);
}

/* ── Soft endstop helper ─────────────────────────────────────────────────────
 * Fades out torque that would push further into the steering lock zone and
 * applies a small counterforce when at or past the hard limit.
 * pos_turns is relative to home (+ = right, − = left).
 * ─────────────────────────────────────────────────────────────────────────── */
static float apply_endstops(float torque, float pos_turns)
{
    float limit = g_config.steering_max_lock / 720.0f;  /* half-range in turns */
    float zone  = limit * 0.08f;   /* fade zone = 8% of half-range             */
    float inner = limit - zone;

    if (pos_turns > inner) {
        /* Right endstop zone: fade out positive torque */
        float t = (pos_turns - inner) / zone;
        if (t > 1.0f) t = 1.0f;
        if (torque > 0.0f) torque *= (1.0f - t);
        /* Hard limit: push back toward centre */
        if (pos_turns >= limit) torque = -g_config.ffb_max_torque * 0.3f;
    } else if (pos_turns < -inner) {
        /* Left endstop zone: fade out negative torque */
        float t = (-pos_turns - inner) / zone;
        if (t > 1.0f) t = 1.0f;
        if (torque < 0.0f) torque *= (1.0f - t);
        if (pos_turns <= -limit) torque = g_config.ffb_max_torque * 0.3f;
    }
    return torque;
}

/* ── hid_apply_ffb ───────────────────────────────────────────────────────────
 * Called every ~10 ms from hid_task_fn.
 * When FFB actuators are enabled:
 *   - switches the controller to TORQUE_CONTROL / PASSTHROUGH
 *   - computes torque from active PID effects
 *   - applies soft endstops
 *   - writes input_torque_ to the ODrive controller
 * When actuators are disabled or USB disconnected: zeroes torque and restores
 * the configured control mode.
 * ─────────────────────────────────────────────────────────────────────────── */
extern "C" void hid_apply_ffb(void)
{
    Axis &ax = axis0();

    /* Only operate in closed loop — motor handles other states itself */
    if (ax.current_state_ != Axis::AXIS_STATE_CLOSED_LOOP_CONTROL) return;

    /* Safety: zero torque if USB disconnected */
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        ax.controller_.input_torque_ = 0.0f;
        return;
    }

    if (ffb_actuators_enabled()) {
        /* Switch to torque control on first FFB enable */
        if (ax.controller_.config_.control_mode !=
                Controller::CONTROL_MODE_TORQUE_CONTROL) {
            ax.controller_.config_.control_mode  = Controller::CONTROL_MODE_TORQUE_CONTROL;
            ax.controller_.config_.input_mode    = Controller::INPUT_MODE_PASSTHROUGH;
            ax.controller_.input_torque_         = 0.0f;
        }

        float pos = ax.encoder_.pos_estimate_.present().value_or(0.0f)
                    - s_home_offset;
        float vel = ax.encoder_.vel_estimate_.present().value_or(0.0f);

        float torque = ffb_compute_torque(pos, vel);
        torque       = apply_endstops(torque, pos);

        ax.controller_.input_torque_ = torque;
    } else {
        /* FFB disabled — restore configured mode and zero torque */
        if (ax.controller_.config_.control_mode ==
                Controller::CONTROL_MODE_TORQUE_CONTROL) {
            ax.controller_.config_.control_mode =
                static_cast<Controller::ControlMode>(g_config.control_mode);
            ax.controller_.input_torque_ = 0.0f;
        }
    }
}

/* ── hid_app_init ────────────────────────────────────────────────────────────── */
extern "C" void hid_app_init(void)
{
    /* M0 encoder remapped to TIM2 on PA0/PA1 (GPIO1/GPIO2 header pins).
     * PA0/PA1 are configured as ENC0 (TIM2_CH1/CH2, AF1) by the ODrive GPIO
     * init loop via DEFAULT_GPIO_MODES — no manual override needed here. */

    /* Initialise FFB state machine (zeroes all effects, disables actuators) */
    ffb_init();

    /* Load config from flash; fall back to factory defaults */
    if (!flash_load_config()) {
        config_init();
    }

    /* Apply loaded config to ODrive live objects.
     * Use app_apply_config_core (no reboot check) because at this point the
     * ODrive live encoder mode is still the default (INCREMENTAL = 0) regardless
     * of what is saved in flash.  Comparing live vs. saved here would always
     * detect a "change" for AMS/SPI modes and cause an infinite reboot loop. */
    app_apply_config_core();

    static const ProtocolCallbacks_t cb = {
        .set_axis_state = app_set_axis_state,
        .set_input_pos  = app_set_input_pos,
        .clear_errors   = app_clear_errors,
        .enter_dfu      = app_enter_dfu,
        .apply_config   = app_apply_config,
        .set_home       = app_set_home,
        .save_config    = app_save_config,
    };
    protocol_init(&cb);
}
