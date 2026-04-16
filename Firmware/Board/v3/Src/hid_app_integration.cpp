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
#include "hid_queue.h"

/* ── Convenience accessor ───────────────────────────────────────────────────── */
static inline Axis& axis0() { return odrv.get_axis(0); }

/* ── Home / zero-position offset (set via CALL_SET_HOME) ────────────────────── */
static float s_home_offset = 0.0f;

/* ── Forward declaration ────────────────────────────────────────────────────── */
static void app_apply_config(void);

/* ── PID State change tracking (set by hid_apply_ffb, consumed by hid_send_telemetry) ── */
static volatile bool s_pid_state_dirty   = true;   /* true at boot → send initial state */
static volatile bool s_pid_actuators_on  = false;

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
    /* FFB operates in torque control mode with vel_gain=0.
     * enable_torque_mode_vel_limit defaults to TRUE in ODrive and would
     * clamp every torque command to zero when vel_gain=0:
     *   Tmax = (vel_limit − vel) × 0 = 0
     *   Tmin = (−vel_limit − vel) × 0 = 0  → torque = clamp(t, 0, 0) = 0
     * Disabling this gate lets our FFB torque reach the motor. */
    ax.controller_.config_.enable_torque_mode_vel_limit = false;

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

    /* Ensure the encoder counter is running (no-op if already started). */
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

    /* ── NPN open-collector encoder signal conditioning ───────────────────────
     * E38S6G5-600B-G24N A/B outputs are NPN open-collector: the line pulls
     * LOW when active and floats high otherwise.  Without an explicit pull-up:
     *   • Rising edges are slow (driven only by line + pin capacitance)
     *   • Motor PWM switching couples readily onto the floating lines
     *   • Both effects produce phantom encoder counts → jittery pos_estimate
     *
     * Fix 1 — GPIO pull-up on PA0 / PA1 (TIM2 CH1/CH2):
     *   Adds the internal ~40 kΩ pull-up to 3.3 V, giving a clean high level
     *   without requiring external resistors on the board.
     *
     * Fix 2 — TIM2 IC input filter (IC1F = IC2F = 0xF):
     *   f_DTS/32, N=8 → rejects glitches shorter than ≈ 1.5 µs.
     *   At 600 PPR × 1000 RPM the minimum inter-edge period is ≈ 100 µs,
     *   so every real edge passes while PWM-coupled spikes are blocked.
     *
     * Applied AFTER HAL_TIM_Encoder_Start so the filter is not overwritten
     * by any HAL routine that re-initialises the capture/compare registers.
     * ──────────────────────────────────────────────────────────────────────── */
    {
        GPIO_InitTypeDef gp = {};
        gp.Pin       = GPIO_PIN_0 | GPIO_PIN_1;  /* PA0 = TIM2_CH1, PA1 = TIM2_CH2 */
        gp.Mode      = GPIO_MODE_AF_PP;
        gp.Pull      = GPIO_PULLUP;              /* ~40 kΩ to 3.3 V                 */
        gp.Speed     = GPIO_SPEED_FREQ_HIGH;
        gp.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &gp);

        /* ── Step 1: TIM CR1 CKD prescaler — divides f_CK_INT before filter
         *
         * TIM2 clock = APB1 timer clock = 84 MHz on ODrive v3 (STM32F405 @ 168 MHz).
         * CKD = 01 → f_DTS = 84 MHz / 2 = 42 MHz.
         *
         * Without CKD prescaler (CKD=00):  f_DTS = 84 MHz
         * With    CKD prescaler (CKD=01):  f_DTS = 42 MHz
         *
         * CR1 bits [9:8] = CKD[1:0].  Safe to change while CEN=1. */
        uint32_t cr1 = htim->Instance->CR1;
        cr1 &= ~(3U << 8U);   /* clear CKD[1:0]   */
        cr1 |=  (1U << 8U);   /* CKD = 01 (÷2)    */
        htim->Instance->CR1 = cr1;

        /* ── Step 2: IC input filter — IC1F = IC2F = 0xF
         *
         * Filter mode 0xF = f_DTS/32, N=8.
         * With CKD=01: f_sample = 42 MHz / 32 = 1.3125 MHz
         * Minimum valid pulse width = N / f_sample = 8 / 1.3125 MHz ≈ 6.1 µs
         *
         * PWM switching transients + cable ringing from motor:  typically 0.2–3 µs → BLOCKED ✓
         * Real encoder edges at 600 PPR × 1000 RPM:  period ≈ 100 µs             → PASSED  ✓
         * Real encoder edges at 600 PPR × 500 RPM:   period ≈ 200 µs             → PASSED  ✓
         *
         * IC1F: CCMR1 bits [7:4],  IC2F: CCMR1 bits [15:12] */
        uint32_t ccmr1 = htim->Instance->CCMR1;
        ccmr1 &= ~((0x0FU << 4U) | (0x0FU << 12U));  /* clear IC1F, IC2F */
        ccmr1 |=   (0x0FU << 4U) | (0x0FU << 12U);   /* set both to 0xF  */
        htim->Instance->CCMR1 = ccmr1;
    }
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

/* ── hid_send_telemetry — called every 10 ms from hid_task_fn ───────────────
 * Queues (in order): PID State (if changed), joystick, telemetry.
 * Then kicks hid_queue_process() so the first item transmits immediately
 * if the endpoint is free.  Subsequent items drain via the DataIn callback.
 * ─────────────────────────────────────────────────────────────────────────── */
extern "C" void hid_send_telemetry(void)
{
    Axis &ax = axis0();

    /* ── Build telemetry payload ── */
    HID_TelemetryPayload_t t;
    memset(&t, 0, sizeof(t));

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
    t.flags            = (ax.encoder_.is_ready_      ? 0x01U : 0U)
                       | (ax.motor_.is_calibrated_   ? 0x02U : 0U);
    t.axis_error       = static_cast<uint32_t>(ax.error_);
    t.motor_error      = static_cast<uint32_t>(ax.motor_.error_);
    t.encoder_error    = static_cast<uint32_t>(ax.encoder_.error_);
    if (g_config.encoder_mode == 0) {
        /* Modo INCREMENTAL: abs_agc_ e abs_diag_flags_ são sempre 0.
         * Sobrescrevemos com os 16 bits baixos do contador do TIM2 para
         * que a GUI possa mostrar o valor bruto e confirmar o CPR real.
         * Mover 1 volta completa deve incrementar em exatamente encoder_cpr. */
        uint16_t raw = (uint16_t)(ax.encoder_.timer_->Instance->CNT & 0xFFFFU);
        t.mag_agc   = (uint8_t)(raw & 0xFFU);
        t.mag_flags = (uint8_t)((raw >> 8U) & 0xFFU);
    } else {
        t.mag_agc   = ax.encoder_.abs_agc_;
        t.mag_flags = ax.encoder_.abs_diag_flags_;
    }

    /* ── PID State (only on actuator enable/disable change) ── */
    if (s_pid_state_dirty) {
        s_pid_state_dirty = false;
        uint8_t pid_report[2];
        pid_report[0] = FFB_REPORT_PID_STATE;
        pid_report[1] = s_pid_actuators_on ? 0x01U : 0x00U;
        hid_queue_push(pid_report, sizeof(pid_report));
    }

    /* ── Joystick (Report 0x01) — consumed by joy.cpl / DirectInput ──
     * IIR low-pass filter on the joystick axis only — does NOT affect FFB or
     * telemetry, which both read pos_estimate_ directly.
     *
     * Why filter here: with a tight steering lock (e.g. 90°) and a 600 PPR
     * (2400 CPR) encoder, one encoder count = 32767/300 ≈ 109 HID axis units.
     * Without smoothing the raw axis jumps 109 units per count in joy.cpl,
     * which looks like noise but is just the quantization amplified by the
     * tight lock.  A gentle 15 Hz filter at 100 Hz cadence adds ~6 ms of
     * lag to the axis — imperceptible in steering but eliminates the jitter.
     *
     * fc = 15 Hz, dt = 10 ms → x = 2π × 15 × 0.01 = 0.942
     * alpha ≈ 1 − exp(−0.942) ≈ 0.610  (first-order Taylor)                */
    // =========================
    // JOYSTICK AXIS (FFB wheel)
    // =========================

    float max_lock = g_config.steering_max_lock;
    if (max_lock < 10.0f) {
        max_lock = 10.0f;
    }

    float half_turns = max_lock / 720.0f;

    float pos = t.pos_estimate;

    // DEADZONE
    const float DEADZONE = 0.0005f;

    static float joy_filt = 0.0f;

    if (fabsf(pos) < DEADZONE) {
        pos = 0.0f;
        joy_filt = 0.0f; // evita drift
    }

    // NORMALIZAÇÃO
    float normalized = pos / half_turns;

    // clamp
    if (normalized > 1.0f)  normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;

    // HID
    float joy_raw = normalized * 32767.0f;

    // FILTRO
    const float ALPHA = 0.3f;
    joy_filt += ALPHA * (joy_raw - joy_filt);

    // CLAMP FINAL
    int16_t joy_out = (int16_t)joy_filt;

    if (joy_out > 32767) joy_out = 32767;
    if (joy_out < -32767) joy_out = -32767;

    // ENVIO
    HID_Joystick_Send(joy_out);

    /* ── Telemetry (Report 0x02) — consumed by the GUI ── */
    HID_ODrive_SendTelemetry(&t);

    /* ── Kick queue: start transmitting if EP is currently idle ── */
    hid_queue_process();
}

/* ── Soft endstop helper ─────────────────────────────────────────────────────
 * Fades out torque that would push further into the steering lock zone and
 * applies a small counterforce when at or past the hard limit.
 * pos_turns is relative to home (+ = right, − = left).
 * ─────────────────────────────────────────────────────────────────────────── */
/* ── Integer power: avoids powf() on Cortex-M4 ─────────────────────────────
 * exp 1=linear, 2=quadratic, 3=cubic, 4=quartic.  t assumed ∈ [0..1].      */
static float _endstop_pow(float t, int32_t exp_i)
{
    float r = t;
    for (int32_t i = 1; i < exp_i && i < 4; i++) r *= t;
    return r;
}

/* ── apply_endstops ──────────────────────────────────────────────────────────
 * Progressive soft endstop model:
 *
 *   inner ──────── fade zone ──────── limit
 *                   ↑                  ↑
 *              fade starts        hard wall
 *
 * In the fade zone:
 *   t ∈ [0..1]  where 0 = inner edge, 1 = hard limit
 *   fade = 1 − t^exp   (configurable exponent: 1=linear, 2=quadratic…)
 *   torque_out_dir *= fade    (only attenuates force pushing further out)
 *
 * At hard limit (t ≥ 1):
 *   Full counterforce: ±ffb_max_torque × ffb_endstop_strength
 *   Direction always toward center.
 *
 * The progressive curve (exp=2 default) means:
 *   • Very little resistance until 80% into the fade zone
 *   • Strongly nonlinear near the limit — feels like hitting a rubber wall
 *   • No abrupt force discontinuity anywhere                                 */
static float apply_endstops(float torque, float pos_turns)
{
    float   limit    = g_config.steering_max_lock / 720.0f;
    float   range    = g_config.ffb_endstop_range;
    float   strength = g_config.ffb_endstop_strength;
    int32_t exp_i    = g_config.ffb_endstop_exp;
    float   zone     = limit * range;
    float   inner    = limit - zone;

    /* Clamp exponent to valid range (should already be validated in config_set) */
    if (exp_i < 1) exp_i = 1;
    if (exp_i > 4) exp_i = 4;

    if (pos_turns > inner) {
        /* ── Right endstop ── */
        if (pos_turns >= limit) {
            /* Past hard limit: full progressive counterforce toward center */
            float excess = pos_turns - limit;           /* turns past limit */
            /* Build-up: counterforce grows linearly with penetration depth  */
            float cf = strength * g_config.ffb_max_torque
                     + excess * g_config.ffb_max_torque; /* progressive     */
            torque = -fminf(cf, g_config.ffb_max_torque);
        } else if (zone > 0.0f) {
            /* In fade zone: attenuate outward torque progressively */
            float t    = (pos_turns - inner) / zone;   /* 0→1 into zone     */
            float fade = 1.0f - _endstop_pow(t, exp_i);
            if (torque > 0.0f) torque *= fade;
            /* Add partial counterforce proportional to penetration */
            torque -= _endstop_pow(t, exp_i)
                    * strength * g_config.ffb_max_torque * 0.5f;
        }
    } else if (pos_turns < -inner) {
        /* ── Left endstop (mirror) ── */
        if (pos_turns <= -limit) {
            float excess = (-pos_turns) - limit;
            float cf = strength * g_config.ffb_max_torque
                     + excess * g_config.ffb_max_torque;
            torque = fminf(cf, g_config.ffb_max_torque);
        } else if (zone > 0.0f) {
            float t    = (-pos_turns - inner) / zone;
            float fade = 1.0f - _endstop_pow(t, exp_i);
            if (torque < 0.0f) torque *= fade;
            torque += _endstop_pow(t, exp_i)
                    * strength * g_config.ffb_max_torque * 0.5f;
        }
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

    bool actuators_on = ffb_actuators_enabled();

    if (actuators_on) {
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

    /* Track state changes so hid_send_telemetry can push PID State report */
    if (actuators_on != s_pid_actuators_on) {
        s_pid_actuators_on = actuators_on;
        s_pid_state_dirty  = true;
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

    /* ── app_ffb_test_on ─────────────────────────────────────────────────────
     * Enables FFB actuators directly from the GUI (no DirectInput game needed).
     * Allows testing spring/endstop/damping effects on the bench.
     * The motor MUST be in CLOSED_LOOP_CONTROL for any torque to be applied.  */
    static auto app_ffb_test_on = []() {
        uint8_t cmd = FFB_DC_ENABLE_ACTUATORS;
        ffb_process_report(FFB_REPORT_DEVICE_CONTROL, &cmd, 1U);
    };

    static auto app_ffb_test_off = []() {
        /* Stop all effects then disable actuators — clean state */
        uint8_t stop = FFB_DC_STOP_ALL_EFFECTS;
        ffb_process_report(FFB_REPORT_DEVICE_CONTROL, &stop, 1U);
        uint8_t dis  = FFB_DC_DISABLE_ACTUATORS;
        ffb_process_report(FFB_REPORT_DEVICE_CONTROL, &dis,  1U);
    };

    static const ProtocolCallbacks_t cb = {
        .set_axis_state = app_set_axis_state,
        .set_input_pos  = app_set_input_pos,
        .clear_errors   = app_clear_errors,
        .enter_dfu      = app_enter_dfu,
        .apply_config   = app_apply_config,
        .set_home       = app_set_home,
        .save_config    = app_save_config,
        .ffb_test_on    = app_ffb_test_on,
        .ffb_test_off   = app_ffb_test_off,
    };
    protocol_init(&cb);

    /* Auto-enter closed-loop control when motor AND encoder are both
     * pre-calibrated.  For a sim racing wheel the motor is always meant to be
     * active — there is no reason to stay in IDLE after a clean boot.
     *
     * Uses the ODrive startup-sequence mechanism (AXIS_STATE_STARTUP_SEQUENCE)
     * with startup_closed_loop_control = true so the axis state machine handles
     * all internal guards (is_calibrated_, error flags, etc.) correctly.
     *
     * If either flag is not set the axis stays in IDLE so the user can still
     * run motor / encoder calibration from the GUI without interference.      */
    if (g_config.motor_pre_calibrated != 0 && g_config.encoder_pre_calibrated != 0) {
        axis0().config_.startup_closed_loop_control = true;
        axis0().requested_state_ = Axis::AXIS_STATE_STARTUP_SEQUENCE;
    }
}
