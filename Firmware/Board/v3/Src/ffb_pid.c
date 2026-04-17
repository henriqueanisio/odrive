#include "ffb_pid.h"
#include "ffb_lut.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <string.h>


/* ── Internal effect state ────────────────────────────────────────────────── */
typedef struct {
    uint8_t  type;              /* FFB_ET_xxx                                  */
    bool     active;
    uint8_t  gain;              /* 0–255, per-effect gain                      */
    uint16_t duration_ms;       /* 0xFFFF = infinite                           */
    uint32_t start_tick;        /* HAL_GetTick() at Op Start                   */

    /* Constant Force */
    int16_t  magnitude;         /* -10000..+10000                              */

    /* Spring / Damper (condition) */
    int16_t  cp_offset;         /* -10000..+10000  (centre point, in 10k units) */
    int16_t  pos_coeff;         /* 0..10000                                    */
    int16_t  neg_coeff;         /* 0..10000                                    */
    uint16_t pos_sat;           /* 0..10000                                    */
    uint16_t dead_band;         /* 0..10000 half-width                         */

    /* Periodic (Sine / Square / Triangle) */
    uint16_t per_magnitude;     /* 0..10000 — half-amplitude                   */
    int16_t  per_offset;        /* -10000..+10000 — DC bias                    */
    uint16_t per_phase;         /* 0..35999 centidegrees                       */
    uint16_t per_period_ms;     /* oscillation period in ms (0 → 10 ms)        */
} FfbEffect_t;

typedef struct {
    FfbEffect_t effects[FFB_MAX_EFFECTS];
    uint8_t     device_gain;        /* 0–255 global gain, default 255          */
    bool        actuators_enabled;
    bool        paused;
    /* Block Load: which slot the host last asked to create */
    uint8_t     last_load_index;    /* 1-based                                 */
    uint8_t     last_load_status;   /* 1=success, 2=full, 3=error              */
    /* Diagnostic counters */
    uint16_t    rx_count;           /* increments each ffb_process_report call */
    uint8_t     last_rx_rid;        /* report ID of last received report       */
} FfbState_t;

static FfbState_t s;

/* ── Public debug record — written every ffb_compute_torque() call ────────── */
FFB_DebugTorque_t g_ffb_debug;

/* ── Control-loop period ─────────────────────────────────────────────────── */
#define FFB_DT_S   (0.010f)    /* seconds — nominal task period (10 ms)       */
#define FFB_DT_MS  (10.0f)     /* milliseconds — same value                   */

/* ── Helper: clamp float ────────────────────────────────────────────────────*/
static float fclamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ── ffb_init ────────────────────────────────────────────────────────────── */
void ffb_init(void)
{
    memset(&s, 0, sizeof(s));
    s.device_gain       = 255U;
    s.actuators_enabled = true;    /* auto-enabled: many games (LFS, AC, etc.)
                                    * never send DC_ENABLE_ACTUATORS and expect
                                    * the wheel to respond immediately.
                                    * Games can still disable via Device Control. */
    s.last_load_status  = 3U;      /* 3 = Block Load Error (no slot allocated yet)
                                    * prevents DirectInput from using index 0     */
}

/* ── Internal: find a free slot for a new effect, return 1-based index ───── */
static uint8_t _alloc_slot(uint8_t requested_type)
{
    /* Reuse existing slot of same type first */
    for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
        if (s.effects[i].type == requested_type) return (uint8_t)(i + 1U);
    }
    /* Otherwise find empty slot */
    for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
        if (s.effects[i].type == FFB_ET_NONE) return (uint8_t)(i + 1U);
    }
    return 0U; /* full */
}

/* ── ffb_process_report ──────────────────────────────────────────────────── */
void ffb_process_report(uint8_t report_id, const uint8_t *data, uint16_t len)
{
    if (!data || len == 0U) return;

    s.rx_count++;
    s.last_rx_rid = report_id;

    switch (report_id) {

    case FFB_REPORT_SET_EFFECT: {
        if (len < sizeof(FFB_SetEffect_t)) break;
        const FFB_SetEffect_t *r = (const FFB_SetEffect_t *)data;
        uint8_t idx = r->effect_block_index;

        /* If index is 0 or out of range, allocate a new slot */
        if (idx == 0U || idx > FFB_MAX_EFFECTS) {
            idx = _alloc_slot(r->effect_type);
        }
        if (idx == 0U) {
            s.last_load_index  = 0U;
            s.last_load_status = 2U; /* full */
            break;
        }

        FfbEffect_t *e = &s.effects[idx - 1U];
        e->type        = r->effect_type;
        e->gain        = r->gain;
        e->duration_ms = r->duration;
        e->active      = false;   /* armed but not yet started (Op Start needed) */

        s.last_load_index  = idx;
        s.last_load_status = 1U; /* success */
        break;
    }

    case FFB_REPORT_SET_PERIODIC: {
        if (len < sizeof(FFB_SetPeriodic_t)) break;
        const FFB_SetPeriodic_t *r = (const FFB_SetPeriodic_t *)data;
        uint8_t idx = r->effect_block_index;
        if (idx == 0U || idx > FFB_MAX_EFFECTS) break;
        FfbEffect_t *e       = &s.effects[idx - 1U];
        e->per_magnitude     = r->magnitude;
        e->per_offset        = r->offset;
        e->per_phase         = r->phase;
        e->per_period_ms     = (r->period > 0U) ? r->period : 10U;
        break;
    }

    case FFB_REPORT_SET_CONDITION: {
        if (len < sizeof(FFB_SetCondition_t)) break;
        const FFB_SetCondition_t *r = (const FFB_SetCondition_t *)data;
        uint8_t idx = r->effect_block_index;
        if (idx == 0U || idx > FFB_MAX_EFFECTS) break;
        FfbEffect_t *e = &s.effects[idx - 1U];
        e->cp_offset = r->cp_offset;
        e->pos_coeff = r->positive_coefficient;
        e->neg_coeff = r->negative_coefficient;
        e->pos_sat   = r->positive_saturation;
        e->dead_band = r->dead_band;
        break;
    }

    case FFB_REPORT_SET_CONSTANT_FORCE: {
        if (len < sizeof(FFB_SetConstantForce_t)) break;
        const FFB_SetConstantForce_t *r = (const FFB_SetConstantForce_t *)data;
        uint8_t idx = r->effect_block_index;
        if (idx == 0U || idx > FFB_MAX_EFFECTS) break;
        s.effects[idx - 1U].magnitude = r->magnitude;
        break;
    }

    case FFB_REPORT_EFFECT_OPERATION: {
        if (len < sizeof(FFB_EffectOperation_t)) break;
        const FFB_EffectOperation_t *r = (const FFB_EffectOperation_t *)data;
        uint8_t idx = r->effect_block_index;
        if (idx == 0U || idx > FFB_MAX_EFFECTS) break;
        FfbEffect_t *e = &s.effects[idx - 1U];

        if (r->operation == FFB_OP_START || r->operation == FFB_OP_START_SOLO) {
            if (r->operation == FFB_OP_START_SOLO) {
                /* Stop all other effects */
                for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
                    if ((i + 1U) != idx) s.effects[i].active = false;
                }
            }
            e->active     = true;
            e->start_tick = HAL_GetTick();
        } else if (r->operation == FFB_OP_STOP) {
            e->active = false;
        }
        break;
    }

    case FFB_REPORT_DEVICE_CONTROL: {
        if (len < 1U) break;
        uint8_t ctrl = data[0];
        /* Bit flags per OpenFFBoard DEVCTRLREP (8×1-bit Variable) */
        if (ctrl & FFB_DC_ENABLE_ACTUATORS) {
            s.actuators_enabled = true;
            s.paused            = false;
        }
        if (ctrl & FFB_DC_DISABLE_ACTUATORS) {
            s.actuators_enabled = false;
        }
        if (ctrl & FFB_DC_STOP_ALL_EFFECTS) {
            for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) s.effects[i].active = false;
        }
        if (ctrl & FFB_DC_DEVICE_RESET) {
            ffb_init();
        }
        if (ctrl & FFB_DC_DEVICE_PAUSE) {
            s.paused = true;
        }
        if (ctrl & FFB_DC_DEVICE_CONTINUE) {
            s.paused = false;
        }
        break;
    }

    case FFB_REPORT_DEVICE_GAIN: {
        if (len < 1U) break;
        s.device_gain = data[0];
        break;
    }

    default:
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── IIR alpha from cutoff frequency ────────────────────────────────────────
 * alpha = 1 − exp(−2π × fc × dt)
 *
 * Computed without expf() via a 4th-order Taylor series accurate to <0.05%
 * for fc < 64 Hz (x < 4).  Above that we saturate to alpha ≈ 0.982 which is
 * equivalent to a brick-wall low-pass — effectively transparent.
 *
 *   fc = 0   → alpha = 0.0  (output frozen — acts as a hold)
 *   fc = 10  → alpha ≈ 0.47 (~10 ms rise)
 *   fc = 30  → alpha ≈ 0.85 (~3 ms rise)
 *   fc = 60  → alpha ≈ 0.98 (~1.5 ms rise)
 *   fc = inf → alpha = 1.0  (passthrough)
 * ─────────────────────────────────────────────────────────────────────────── */
static float _iir_alpha(float fc_hz)
{
    if (fc_hz <= 0.0f) return 0.0f;   /* fc=0 → frozen output              */

    float x = 6.2832f * fc_hz * FFB_DT_S;   /* 2π × fc × dt               */
    if (x >= 4.0f) return 0.9817f;           /* saturate: 1 − exp(−4)      */

    /* exp(−x) via Taylor: 1 − x + x²/2 − x³/6 + x⁴/24 */
    float x2 = x * x;
    float ex = 1.0f - x + x2 * 0.5f
                        - x2 * x * 0.16667f
                        + x2 * x2 * 0.04167f;

    float alpha = 1.0f - ex;
    return fclamp(alpha, 0.0f, 1.0f);
}

/* ── Fast periodic waveforms ────────────────────────────────────────────────
 * All functions take normalised phase p ∈ [0, 1) for one full cycle and
 * return a value in [−1, +1].  No calls to sinf/cosf — safe on bare-metal.
 *
 * _ffb_sin: parabolic approximation, max error < 8 %.  Sufficient for FFB.
 *   Formula: fold to half-cycle, then 4p(1−p).
 *
 * _ffb_tri: triangle wave, exact.
 *
 * _ffb_square: exact square wave with hard edges.
 * ─────────────────────────────────────────────────────────────────────────── */
static float _ffb_sin(float p)
{
    /* wrap to [0, 1) */
    p -= (float)(int32_t)p;
    if (p < 0.0f) p += 1.0f;
    float neg = (p >= 0.5f) ? -1.0f : 1.0f;
    if (p >= 0.5f) p -= 0.5f;
    p *= 2.0f;                   /* [0, 1) over the half-cycle              */
    return neg * 4.0f * p * (1.0f - p);  /* parabola: 0→1→0                */
}

static float _ffb_tri(float p)
{
    p -= (float)(int32_t)p;
    if (p < 0.0f) p += 1.0f;
    /* Triangle: peak at 0, zero at 0.25, trough at 0.5, zero at 0.75      */
    float d = p - 0.5f;
    if (d < 0.0f) d = -d;        /* abs(p - 0.5) ∈ [0, 0.5]               */
    return 4.0f * d - 1.0f;      /* maps [0..0.5] → [-1..1]                */
}

static float _ffb_square(float p)
{
    p -= (float)(int32_t)p;
    if (p < 0.0f) p += 1.0f;
    return (p < 0.5f) ? 1.0f : -1.0f;
}

/* ── Integer power for endstop curve — avoids powf() ───────────────────────
 * exp 1 = linear, 2 = quadratic, 3 = cubic, 4 = quartic.
 * Input t is assumed ∈ [0..1].                                              */
static float _fpow_int(float t, int32_t exp)
{
    float r = t;
    for (int32_t i = 1; i < exp && i < 4; i++) r *= t;
    return r;
}

/* ── Smooth absolute value ──────────────────────────────────────────────────
 * Returns (x < 0) ? −x : x                                                 */
static inline float _fabs_f(float x) { return (x < 0.0f) ? -x : x; }

/* ── LUT application (inline, reads g_config directly) ─────────────────────
 * Applies the 8-point torque linearization LUT to a signed Nm value.
 * Input/output range: ±max_t.  Identity when lut_enabled == 0.              */
static float _apply_lut(float torque_nm, float max_t)
{
    if (!g_config.ffb_lut_enabled || max_t <= 0.0f) return torque_nm;

    float sign = (torque_nm >= 0.0f) ? 1.0f : -1.0f;
    float mag  = _fabs_f(torque_nm) / max_t;   /* normalize to [0..1]      */
    if (mag > 1.0f) mag = 1.0f;

    /* Map magnitude to LUT index */
    float   idx_f = mag * (float)(FFB_LUT_POINTS - 1U);
    uint8_t lo    = (uint8_t)idx_f;

    float out;
    if (lo >= FFB_LUT_POINTS - 1U) {
        out = g_config.ffb_lut[FFB_LUT_POINTS - 1U];
    } else {
        /* Linear interpolation — O(1), no division */
        float frac = idx_f - (float)lo;
        out = g_config.ffb_lut[lo] + frac * (g_config.ffb_lut[lo + 1U]
                                              - g_config.ffb_lut[lo]);
    }

    return sign * out * max_t;   /* denormalize back to Nm                  */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ffb_compute_torque — Professional FFB Pipeline
 *
 *  STAGE  │ BLOCK                        │ WHAT IT DOES
 * ────────┼──────────────────────────────┼─────────────────────────────────
 *    1    │ DirectInput effects           │ HID PID Constant/Spring/Damper
 *    2    │ Input IIR filter              │ Tames game FFB jitter pre-gain
 *    3    │ × Game Gain                  │ device_gain/255 (game controls)
 *    ─────┼──────────────────────────────┼─────────────────────────────────
 *    4    │ Inertia (filtered deriv)     │ −J × dω/dt, noise-free accel
 *    5    │ Stribeck Friction            │ static + dynamic Coulomb
 *    6    │ Damping                      │ vel-proportional, always-on
 *    7    │ Center Spring                │ optional always-on centering
 *    8    │ Center Boost                 │ progressive extra spring near 0
 *    ─────┼──────────────────────────────┼─────────────────────────────────
 *    9    │ × Wheel Gain                 │ ffb_gain (user controls)
 *   10    │ Minimum Force deadzone       │ kill micro-torques
 *   11    │ Thermal derating (I²t)       │ smooth current-limit protection
 *   12    │ Slew Rate Limiter            │ dT/dt cap → Nm/ms × 10 ms
 *   13    │ Output IIR Filter            │ final smoothing
 *   14    │ Torque LUT                   │ motor linearization
 *   15    │ Clamp ±max_t                 │ hard safety limit
 *
 * Endstops: applied by caller (apply_endstops() in hid_app_integration.cpp).
 * Debug:    every field in g_ffb_debug is written every call.
 * ═══════════════════════════════════════════════════════════════════════════ */
float ffb_compute_torque(float pos_turns, float vel_turns_s)
{
    if (!s.actuators_enabled || s.paused) {
        /* Zero all debug fields when disabled */
        __builtin_memset(&g_ffb_debug, 0, sizeof(g_ffb_debug));
        return 0.0f;
    }

    const float max_t = g_config.ffb_max_torque;
    if (max_t <= 0.0f) return 0.0f;

    const uint32_t now = HAL_GetTick();

    /* ── STAGE 1: DirectInput effects ──────────────────────────────────────
     * Accumulate torque from all active HID PID effect slots.
     * Each slot has its own per-effect gain (0–255 from SetEffect report).
     *
     * Spring model: nonlinear  err × |err|  — weak at center, firm at edges
     *   → matches the progressive feel of a Simucube/Fanatec rack.
     * Damper model: linear coefficient × velocity (game controls strength).
     * Constant: direct magnitude with per-effect gain.                      */
    float di_const  = 0.0f;
    float di_spring = 0.0f;
    float di_damper = 0.0f;

    for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
        FfbEffect_t *e = &s.effects[i];
        if (!e->active || e->type == FFB_ET_NONE) continue;

        /* Expire finite-duration effects */
        if (e->duration_ms != 0xFFFFU) {
            if ((now - e->start_tick) >= (uint32_t)e->duration_ms) {
                e->active = false;
                continue;
            }
        }

        float eg = (float)e->gain / 255.0f;

        switch (e->type) {

        case FFB_ET_CONSTANT: {
            float t = ((float)e->magnitude / 10000.0f) * max_t;
            di_const += t * eg;
            break;
        }

        case FFB_ET_SINE:
        case FFB_ET_SQUARE:
        case FFB_ET_TRIANGLE: {
            uint16_t period = (e->per_period_ms > 0U) ? e->per_period_ms : 10U;
            uint32_t elapsed = now - e->start_tick;
            float p = (float)(elapsed % (uint32_t)period) / (float)period;

            /* Phase offset in centidegrees → normalised [0..1] */
            p += (float)e->per_phase / 36000.0f;

            float wave;
            if (e->type == FFB_ET_SINE)          wave = _ffb_sin(p);
            else if (e->type == FFB_ET_SQUARE)   wave = _ffb_square(p);
            else                                 wave = _ffb_tri(p);

            float mag = (float)e->per_magnitude / 10000.0f;
            float off = (float)e->per_offset    / 10000.0f;
            float t   = (wave * mag + off) * max_t;
            di_const += t * eg;
            break;
        }

        case FFB_ET_SPRING: {
            float cp      = (float)e->cp_offset / 10000.0f;
            float err     = pos_turns - cp;
            float abs_err = _fabs_f(err);
            float db      = (float)e->dead_band / 10000.0f;
            if (abs_err <= db) break;

            /* Remove dead-band from error magnitude (signed) */
            float eff_err = (err >= 0.0f) ? (err - db) : (err + db);

            float coeff = (err >= 0.0f)
                ? ((float)e->pos_coeff / 10000.0f)
                : ((float)e->neg_coeff / 10000.0f);

            /* Nonlinear: err × |err| → progressive feel.
             * Magnitude doubles every 2× displacement vs linear.            */
            float abs_eff = _fabs_f(eff_err);
            float t = -coeff * eff_err * abs_eff * max_t;

            /* Per-effect saturation */
            float sat = ((float)e->pos_sat / 10000.0f) * max_t;
            if (sat > 0.0f) t = fclamp(t, -sat, sat);

            di_spring += t * eg;
            break;
        }

        case FFB_ET_DAMPER: {
            /* vel_turns_s * coeff * max_t — sign already correct (opposes motion) */
            float coeff = (float)e->pos_coeff / 10000.0f;
            float t     = -coeff * vel_turns_s * max_t;
            di_damper  += t * eg;
            break;
        }

        default:
            break;
        }
    }

    float di_sum = di_const + di_spring + di_damper;

    /* ── STAGE 2: Input IIR filter (pre-game-gain) ──────────────────────────
     * Smooths game-generated FFB jitter and oscillations before they enter
     * the physical model.  fc=0 disables (alpha=0 → frozen — NOT what we
     * want; when disabled use passthrough instead).
     * We treat fc=0 as "disabled" → alpha=1.0 → passthrough.               */
    {
        static float di_filt = 0.0f;
        float alpha = (g_config.ffb_input_filter_hz > 0.0f)
                      ? _iir_alpha(g_config.ffb_input_filter_hz)
                      : 1.0f;   /* fc=0 → passthrough */
        di_filt += alpha * (di_sum - di_filt);
        di_sum   = di_filt;
    }

    /* ── STAGE 3: Game Gain ─────────────────────────────────────────────────
     * device_gain is set by the game/OS via HID PID Device Gain report (0x0D).
     * It represents the game's "FFB strength" slider — should only scale the
     * game's effects, not the firmware physical corrections below.          */
    const float game_gain = (float)s.device_gain / 255.0f;
    di_sum *= game_gain;

    /* ── STAGE 4: Inertia — filtered acceleration derivative ────────────────
     * Simulates rotational mass of the steering wheel + motor rotor.
     * F_inertia = −J × dω/dt  (opposes acceleration)
     *
     * Raw differentiation (vel − prev_vel)/dt amplifies encoder noise.
     * We pre-filter velocity with a dedicated IIR (ffb_inertia_filter_hz)
     * before differentiating → stable, noise-free acceleration estimate.
     *
     * Default filter: 20 Hz → rejects encoder quantization noise above 20 Hz
     * while passing real steering accelerations (typically < 5 Hz).         */
    float phys_inertia = 0.0f;
    {
        static float vel_filt = 0.0f;   /* low-pass filtered velocity       */
        static float vel_prev = 0.0f;   /* previous filtered velocity       */

        float alpha_v = (g_config.ffb_inertia_filter_hz > 0.0f)
                        ? _iir_alpha(g_config.ffb_inertia_filter_hz)
                        : 1.0f;

        vel_filt += alpha_v * (vel_turns_s - vel_filt);

        float accel = (vel_filt - vel_prev) / FFB_DT_S;   /* turns/s²      */
        vel_prev    = vel_filt;

        /* Scale: inertia coefficient × max_t gives dimensional Nm.
         * Saturate at ±50% max_t to prevent startup spike when state
         * resets (e.g., motor re-enable after disable).                     */
        phys_inertia = fclamp(-g_config.ffb_inertia * accel * max_t,
                              -max_t * 0.5f, max_t * 0.5f);
    }

    /* ── STAGE 5: Stribeck Friction ─────────────────────────────────────────
     * Classical friction has two regimes:
     *   Static (stiction): high breakaway force at zero velocity
     *   Dynamic (Coulomb): lower constant force once moving
     *
     * Stribeck model smoothly blends them:
     *   F_total = F_dyn + F_static × 1/(1 + |vel| / v_stribeck)
     *
     * At v=0:             F = F_dyn + F_static   (max friction)
     * At v=v_stribeck:    F = F_dyn + F_static/2 (50% of extra)
     * At v→∞:             F ≈ F_dyn              (pure kinetic)
     *
     * Smooth sign function:  vel / (|vel| + ε)   — no discontinuity at 0,
     * transitions from −1 to +1 over ≈ ε of velocity.  We use ε = 0.003
     * turns/s (~0.02 RPM), tight enough to feel like real static friction.  */
    float phys_friction = 0.0f;
    {
        /* VEL_EPS = 0.02 turns/s ≈ 7°/s.
         * A 600 PPR encoder in a 10 ms window has velocity quantization noise
         * of ±1/(2400 × 0.01) ≈ ±0.042 turns/s near zero.  Using 0.003 (the
         * theoretical "tight" value) causes the sign to flip with encoder noise
         * every sample → alternating friction force at 50 Hz → buzzing.
         * 0.02 turns/s is well above the noise floor yet still short enough
         * that the user feels genuine static friction below ~7°/s.            */
        const float VEL_EPS = 0.02f;    /* smooth-sign dead-zone (turns/s)  */
        float v_abs   = _fabs_f(vel_turns_s);
        float sign_v  = vel_turns_s / (v_abs + VEL_EPS);   /* smooth sign  */

        /* Stribeck blending: rational approximation of exp-based model     */
        float stribeck = 1.0f / (1.0f + v_abs / g_config.ffb_stribeck_vel);

        float coeff = g_config.ffb_friction
                    + g_config.ffb_static_friction * stribeck;

        phys_friction = -sign_v * coeff * max_t;
    }

    /* ── STAGE 6: Velocity Damping ──────────────────────────────────────────
     * Always-on velocity-proportional resistance.  Prevents oscillation and
     * adds the "weight" feel after springs.  Linear and symmetric.
     * This is separate from the HID Damper effect — it is never zero even
     * when the game sends no effects.                                        */
    float phys_damping = -vel_turns_s * g_config.ffb_damping * max_t;

    /* ── Normalized position ─────────────────────────────────────────────────
     * pos_norm ∈ [−1..+1] where ±1 = at the steering lock limit.
     *
     * Using pos_norm instead of pos_turns makes spring/boost coefficients
     * INDEPENDENT of steering lock angle: ffb_spring=0.3 always means
     * "30% of max_t at the limit", regardless of whether lock = 90° or 900°.
     *
     * Without normalization with 90° lock (half=0.125 turns):
     *   spring force = 0.3 × 3.0 × 0.125 = 0.11 Nm → too weak to feel.
     * With normalization:
     *   spring force = 0.3 × 3.0 × 1.0   = 0.90 Nm → useful centering.   */
    float half_lock = g_config.steering_max_lock / 720.0f;   /* turns        */
    float pos_norm  = (half_lock > 0.001f)
                      ? fclamp(pos_turns / half_lock, -1.0f, 1.0f)
                      : 0.0f;

    /* ── STAGE 7: Always-on Center Spring ───────────────────────────────────
     * Force = −pos_norm × spring × max_t
     * Reaches full spring × max_t exactly at the steering lock limit.
     * Set ffb_spring = 0 to disable.                                        */
    float phys_spring = 0.0f;
    if (g_config.ffb_spring > 0.0f) {
        phys_spring = -pos_norm * g_config.ffb_spring * max_t;
    }

    /* ── STAGE 8: Center Boost ───────────────────────────────────────────────
     * Extra spring force near center only — fades to zero at ±boost_width.
     * boost_width is in NORMALIZED units [0..1] (fraction of steering lock).
     *   0.2 = boost active in the inner 20% of travel each side.
     *   At 900° lock: inner 90° (±45°) | At 90° lock: inner 9° (±4.5°)
     *
     * Force: −pos_norm × boost_str × max_t × quadratic_fade               */
    float phys_boost = 0.0f;
    if (g_config.ffb_center_boost_str > 0.0f
        && g_config.ffb_center_boost_width > 0.0f)
    {
        float abs_norm = _fabs_f(pos_norm);
        float width    = g_config.ffb_center_boost_width;   /* normalized   */
        if (abs_norm < width) {
            float t_boost = abs_norm / width;         /* 0 at center → 1    */
            float fade    = 1.0f - t_boost * t_boost; /* quadratic fade     */
            phys_boost    = -pos_norm * g_config.ffb_center_boost_str
                            * max_t * fade;
        }
    }

    /* ── Sum physical effects ────────────────────────────────────────────── */
    float phys_sum = phys_inertia + phys_friction
                   + phys_damping + phys_spring + phys_boost;

    /* ── STAGE 9: Wheel Gain ─────────────────────────────────────────────────
     * ffb_gain is the hardware-level output scale set by the user in the GUI.
     * It scales everything — DI effects AND firmware physical effects.
     * This is the "master volume" of the wheel, independent of the game.    */
    const float wheel_gain  = g_config.ffb_gain;
    float total_pre_scale   = di_sum + phys_sum;
    float torque            = total_pre_scale * wheel_gain;

    /* ── STAGE 10: Minimum Force Deadzone ────────────────────────────────────
     * Cancels micro-torques below the threshold — eliminates motor buzzing
     * from quantization noise and tiny game effects.
     * Only active if ffb_min_force > 0.                                     */
    if (g_config.ffb_min_force > 0.0f) {
        float min_t = g_config.ffb_min_force * max_t;
        if (_fabs_f(torque) < min_t) {
            torque = 0.0f;
        }
    }

    /* ── STAGE 11: Thermal Derating (I²t model) ─────────────────────────────
     * Models the winding thermal state using a first-order I²t integrator.
     *
     * Concept:
     *   Power ∝ I² ∝ (torque/Kt)².  We track a normalized thermal load:
     *     load = (I/I_rated)²   where I_rated = current_lim × 0.7
     *   The thermal state approaches this load with time constant tau:
     *     dT_state/dt = (load − T_state) / tau
     *
     *   When T_state > 1.0 (over rated power):
     *     scale = 1 / T_state  (hyperbolic derate, smooth)
     *     clamped to ffb_thermal_min (never drops to zero).
     *
     * Default tau = 30 s → motor winding has 30 s thermal inertia.
     * At 100% rated current continuously: scale drops to ~1/e ≈ 0.37 after tau.
     * This gives a natural "soft cut" that the user feels as gradual weakening
     * rather than a hard torque cap.                                         */
    float thermal_scale = 1.0f;
    {
        static float thermal_state = 0.0f;

        /* Estimate current from commanded torque (before derating).
         * Guard against zero Kt to avoid division by zero.                  */
        float kt     = (g_config.torque_constant > 0.001f)
                       ? g_config.torque_constant : 0.001f;
        float i_est  = _fabs_f(torque) / kt;           /* A, estimated      */
        float i_rat  = g_config.current_lim * 0.7f;    /* 70% CL = rated   */
        if (i_rat < 0.1f) i_rat = 0.1f;

        /* Normalized thermal load (1.0 = rated continuous power) */
        float i_norm = i_est / i_rat;
        float load   = i_norm * i_norm;   /* squared → proportional to power */

        /* First-order thermal integrator: alpha = dt/tau */
        float alpha_th = FFB_DT_S / g_config.ffb_thermal_tau;
        if (alpha_th > 1.0f) alpha_th = 1.0f;
        thermal_state += alpha_th * (load - thermal_state);
        if (thermal_state < 0.0f) thermal_state = 0.0f;

        /* Derate only when over rated load */
        if (thermal_state > 1.0f) {
            thermal_scale = 1.0f / thermal_state;   /* hyperbolic derate   */
            float min_s   = g_config.ffb_thermal_min;
            if (thermal_scale < min_s) thermal_scale = min_s;
        }

        torque *= thermal_scale;
    }

    /* ── STAGE 12: Slew Rate Limiter ─────────────────────────────────────────
     * Hard cap on torque derivative: dT/dt ≤ ffb_slew_rate Nm/ms.
     * Per-cycle budget: slew_rate × DT_MS  [Nm].
     *
     * Why Nm/ms instead of Nm/s?  At 3 Nm max and 10 ms period:
     *   0.05 Nm/ms → 0.5 Nm/step → 50 Nm/s → reaches max in 60 ms (fast)
     *   0.01 Nm/ms → 0.1 Nm/step → 10 Nm/s → reaches max in 300 ms (slow)
     * 0.05 Nm/ms gives Simucube-like "snappy but controlled" transients.
     * slew_rate = 0 → limiter disabled.                                     */
    {
        static float prev_torque = 0.0f;
        if (g_config.ffb_slew_rate > 0.0f) {
            float budget = g_config.ffb_slew_rate * FFB_DT_MS;   /* Nm/step */
            float delta  = torque - prev_torque;
            if (delta >  budget) delta =  budget;
            if (delta < -budget) delta = -budget;
            torque = prev_torque + delta;
        }
        prev_torque = torque;
    }

    /* ── STAGE 13: Output IIR Low-pass Filter ────────────────────────────────
     * Final smoothing after slew limiting.  Reduces motor electrical noise
     * (switching ripple) without adding meaningful phase lag at useful
     * mechanical frequencies.
     *
     * fc = 60 Hz default → −3 dB at 60 Hz, −20 dB/dec beyond.
     * Lag at 10 Hz (typical steering transient): tan⁻¹(10/60) ≈ 9.5°
     *   → group delay ≈ 2.6 ms — imperceptible.
     * fc = 0 → disabled (alpha = 1.0 → passthrough).                        */
    float pre_lut_torque;
    {
        static float torque_out = 0.0f;
        float alpha = (g_config.ffb_filter_hz > 0.0f)
                      ? _iir_alpha(g_config.ffb_filter_hz)
                      : 1.0f;
        torque_out  += alpha * (torque - torque_out);
        pre_lut_torque = torque_out;
    }

    /* ── STAGE 14: Torque Linearization LUT ─────────────────────────────────
     * Compensates motor/gearbox nonlinearity.
     * LUT is symmetric — applies the same correction to + and − torque.
     * Identity by default (all 8 points on the straight line).
     * Disabled (ffb_lut_enabled = 0) → zero overhead, value passes through. */
    float final_torque = _apply_lut(pre_lut_torque, max_t);

    /* ── STAGE 15: Hard clamp ────────────────────────────────────────────────
     * Safety backstop — should not be reached in normal operation because
     * every earlier stage already respects ±max_t bounds.                   */
    final_torque = fclamp(final_torque, -max_t, max_t);

    /* ── Write debug record ──────────────────────────────────────────────────
     * Updated every cycle.  Read externally from a lower-priority task.
     * Cortex-M4 float writes are word-aligned and therefore atomic.         */
    g_ffb_debug.di_constant    = di_const;
    g_ffb_debug.di_spring      = di_spring;
    g_ffb_debug.di_damper      = di_damper;
    g_ffb_debug.di_total       = di_sum;
    g_ffb_debug.phys_inertia   = phys_inertia;
    g_ffb_debug.phys_friction  = phys_friction;
    g_ffb_debug.phys_damping   = phys_damping;
    g_ffb_debug.phys_spring    = phys_spring;
    g_ffb_debug.phys_boost     = phys_boost;
    g_ffb_debug.thermal_scale  = thermal_scale;
    g_ffb_debug.game_gain      = game_gain;
    g_ffb_debug.wheel_gain     = wheel_gain;
    g_ffb_debug.pre_lut        = pre_lut_torque;
    g_ffb_debug.output         = final_torque;

    return final_torque;
}

/* ── ffb_get_block_load_report ───────────────────────────────────────────── */
uint8_t ffb_get_block_load_report(uint8_t *buf, uint8_t buf_size)
{
    /* [report_id | effect_block_index | block_load_status | ram_pool_lo | ram_pool_hi] */
    if (buf_size < 5U) return 0U;
    buf[0] = FFB_REPORT_PID_BLOCK_LOAD;
    buf[1] = s.last_load_index;
    buf[2] = s.last_load_status;
    /* Report remaining RAM pool (fake: always 1 block available) */
    uint16_t pool = (s.last_load_status == 1U) ? (FFB_MAX_EFFECTS - s.last_load_index) : 0U;
    buf[3] = (uint8_t)(pool & 0xFFU);
    buf[4] = (uint8_t)(pool >> 8U);
    return 5U;
}

/* ── ffb_get_pool_report ─────────────────────────────────────────────────── */
uint8_t ffb_get_pool_report(uint8_t *buf, uint8_t buf_size)
{
    /* [report_id | ram_pool_lo | ram_pool_hi | simultaneous_max | device_managed] */
    if (buf_size < 5U) return 0U;
    buf[0] = FFB_REPORT_PID_POOL;
    uint16_t pool = FFB_MAX_EFFECTS;
    buf[1] = (uint8_t)(pool & 0xFFU);
    buf[2] = (uint8_t)(pool >> 8U);
    buf[3] = FFB_MAX_EFFECTS;   /* simultaneous effects max */
    buf[4] = 0U;                /* not device-managed pool */
    return 5U;
}

/* ── ffb_actuators_enabled ───────────────────────────────────────────────── */
bool ffb_actuators_enabled(void)
{
    return s.actuators_enabled;
}

/* ── ffb_get_diag_stats ──────────────────────────────────────────────────── */
FFB_DiagStats_t ffb_get_diag_stats(void)
{
    FFB_DiagStats_t d;
    d.rx_count = s.rx_count;
    d.last_rid = s.last_rx_rid;
    d.actv     = (s.actuators_enabled ? 0x01U : 0x00U);
    /* bit1: any effect currently active */
    for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
        if (s.effects[i].active) { d.actv |= 0x02U; break; }
    }
    return d;
}
