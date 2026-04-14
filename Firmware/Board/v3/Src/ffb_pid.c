#include "ffb_pid.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>


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
} FfbEffect_t;

typedef struct {
    FfbEffect_t effects[FFB_MAX_EFFECTS];
    uint8_t     device_gain;        /* 0–255 global gain, default 255          */
    bool        actuators_enabled;
    bool        paused;
    /* Block Load: which slot the host last asked to create */
    uint8_t     last_load_index;    /* 1-based                                 */
    uint8_t     last_load_status;   /* 1=success, 2=full, 3=error              */
} FfbState_t;

static FfbState_t s;

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
    s.actuators_enabled = false;   /* enabled only after DC_ENABLE_ACTUATORS   */
    s.last_load_status  = 1U;      /* success */
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
        if (len < sizeof(FFB_DeviceControl_t)) break;
        uint8_t ctrl = data[0];
        switch (ctrl) {
            case FFB_DC_ENABLE_ACTUATORS:
                s.actuators_enabled = true;
                s.paused            = false;
                break;
            case FFB_DC_DISABLE_ACTUATORS:
                s.actuators_enabled = false;
                break;
            case FFB_DC_STOP_ALL_EFFECTS:
                for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) s.effects[i].active = false;
                break;
            case FFB_DC_DEVICE_RESET:
                ffb_init();
                break;
            case FFB_DC_DEVICE_PAUSE:
                s.paused = true;
                break;
            case FFB_DC_DEVICE_CONTINUE:
                s.paused = false;
                break;
            default: break;
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

/* ── ffb_compute_torque ──────────────────────────────────────────────────── */
/*
 * Pipeline de efeitos — ordem de aplicação:
 *   1. Efeitos DirectInput (Constant Force, Spring, Damper via HID PID)
 *   2. Inércia          (-J × Δvel/dt — simula massa do volante)
 *   3. Fricção          (Coulomb: -sign(vel) × friction × max_t)
 *   4. Amortecimento    (vel-proporcional: -vel × damping × max_t)
 *   5. Mola central     (sempre ativa se ffb_spring > 0)
 *   6. Gain global      (device_gain × ffb_gain)
 *   7. Força mínima     (dead-zone: zera torques abaixo de min_force × max_t)
 *   8. Slew rate        (limita variação de torque por ciclo → Nm/ms × 10ms)
 *   9. Filtro passa-baixa (IIR 1ª ordem, alpha calculado a partir de filter_hz)
 *
 * Endstops são aplicados pelo caller (apply_endstops em hid_app_integration.cpp).
 * Período de ciclo nominal: DT_S = 10 ms.
 */
#define FFB_DT_S   (0.010f)   /* seconds per control cycle (10 ms task)      */
#define FFB_DT_MS  (10.0f)    /* same in milliseconds                        */

float ffb_compute_torque(float pos_turns, float vel_turns_s)
{
    if (!s.actuators_enabled || s.paused) return 0.0f;

    const float max_t     = g_config.ffb_max_torque;
    const float gain_norm = (float)s.device_gain / 255.0f;
    const uint32_t now    = HAL_GetTick();

    /* ── PASSO 1: Efeitos DirectInput ──────────────────────────────────────
     * Soma contribuição de cada slot de efeito ativo dentro do prazo.      */
    float torque = 0.0f;

    for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
        FfbEffect_t *e = &s.effects[i];
        if (!e->active || e->type == FFB_ET_NONE) continue;

        /* Expira efeito com duração finita */
        if (e->duration_ms != 0xFFFFU) {
            if ((now - e->start_tick) >= (uint32_t)e->duration_ms) {
                e->active = false;
                continue;
            }
        }

        float eg = (float)e->gain / 255.0f;  /* per-effect gain 0..1 */

        switch (e->type) {

        case FFB_ET_CONSTANT: {
            /* Força constante linear: direta, sem não-linearidade */
            float t = ((float)e->magnitude / 10000.0f) * max_t;
            torque += t * eg;
            break;
        }

        case FFB_ET_SPRING: {
            /* Spring (Condition report): centraliza o volante no cp_offset.
             * Modelo não-linear: err×|err| → progressividade Simucube.     */
            float cp    = (float)e->cp_offset / 10000.0f;
            float err   = pos_turns - cp;
            float db    = (float)e->dead_band / 10000.0f;

            /* Dead-band: sem força enquanto err dentro da zona neutra */
            float abs_err = (err < 0.0f) ? -err : err;
            if (abs_err <= db) break;

            /* Cômputa coeff assimétrico (positivo/negativo) */
            float coeff = (err >= 0.0f)
                ? ((float)e->pos_coeff / 10000.0f)
                : ((float)e->neg_coeff / 10000.0f);

            /* Não-linear: err×|err| dá sensação progressiva semelhante ao
             * Simucube — fraco perto do centro, firme nos extremos          */
            float t = -coeff * err * abs_err * max_t;

            /* Saturação de saída */
            float sat = ((float)e->pos_sat / 10000.0f) * max_t;
            if (sat > 0.0f) t = fclamp(t, -sat, sat);

            torque += t * eg;
            break;
        }

        case FFB_ET_DAMPER: {
            /* Damper (Condition report): resistência proporcional à velocidade.
             * coeff vem do positive_coefficient do report.                   */
            float coeff = (float)e->pos_coeff / 10000.0f;
            float t     = -coeff * vel_turns_s * max_t;
            torque += t * eg;
            break;
        }

        default:
            break;
        }
    }

    /* ── PASSO 2: Inércia (massa do volante simulada) ───────────────────────
     * F_inertia = -J × dω/dt
     * Calculamos dω/dt = (vel_atual - vel_anterior) / dt
     * Isso resiste a mudanças bruscas de velocidade → sensação de peso real. */
    {
        static float last_vel = 0.0f;
        float accel = (vel_turns_s - last_vel) / FFB_DT_S;   /* turns/s²    */
        last_vel = vel_turns_s;

        float inertia_t = -g_config.ffb_inertia * accel * max_t;
        /* Satura inércia em ±50% do max para evitar spike em startup        */
        inertia_t = fclamp(inertia_t, -max_t * 0.5f, max_t * 0.5f);
        torque += inertia_t;
    }

    /* ── PASSO 3: Fricção de Coulomb ────────────────────────────────────────
     * Resistência constante que sempre opõe o movimento.
     * Dead-band em ±0.005 turns/s (~0.03 RPM) para não travar no repouso.   */
    if (g_config.ffb_friction > 0.0f) {
        const float VEL_DEADBAND = 0.005f;
        float friction_t = 0.0f;
        if (vel_turns_s > VEL_DEADBAND) {
            friction_t = -g_config.ffb_friction * max_t;
        } else if (vel_turns_s < -VEL_DEADBAND) {
            friction_t =  g_config.ffb_friction * max_t;
        }
        torque += friction_t;
    }

    /* ── PASSO 4: Amortecimento global ─────────────────────────────────────
     * Proporcional à velocidade — suaviza oscilações (rubber-banding).       */
    torque -= vel_turns_s * g_config.ffb_damping * max_t;

    /* ── PASSO 5: Mola central sempre ativa ────────────────────────────────
     * Independente dos efeitos do jogo — recentra o volante.
     * Útil para menus e quando o jogo não envia spring.                      */
    if (g_config.ffb_spring > 0.0f) {
        torque -= pos_turns * g_config.ffb_spring * max_t;
    }

    /* ── PASSO 6: Gain global ───────────────────────────────────────────────
     * device_gain: controlado pelo jogo via HID PID Device Gain report (0xD).
     * ffb_gain: configurado pelo usuário na GUI.                              */
    torque *= gain_norm * g_config.ffb_gain;

    /* ── PASSO 7: Força mínima (dead-zone) ─────────────────────────────────
     * Remove micro-torques que apenas causam vibração sem feedback útil.
     * min_force define a fração de max_t abaixo da qual a saída é zerada.   */
    if (g_config.ffb_min_force > 0.0f) {
        float min_t = g_config.ffb_min_force * max_t;
        float abs_t = (torque < 0.0f) ? -torque : torque;
        if (abs_t < min_t) {
            torque = 0.0f;
        }
    }

    /* ── PASSO 8: Slew rate limiter ─────────────────────────────────────────
     * Limita quanto o torque pode mudar por ciclo (Nm/ms × 10 ms/ciclo).
     * Previne spikes elétricos e protege o motor de transientes abruptos.
     * slew_rate == 0 → limitador desabilitado.                               */
    {
        static float last_torque = 0.0f;
        if (g_config.ffb_slew_rate > 0.0f) {
            float max_delta = g_config.ffb_slew_rate * FFB_DT_MS;  /* Nm/ms × ms */
            float delta     = torque - last_torque;
            if (delta >  max_delta) delta =  max_delta;
            if (delta < -max_delta) delta = -max_delta;
            torque = last_torque + delta;
        }
        last_torque = torque;
    }

    /* ── PASSO 9: Filtro passa-baixa (IIR 1ª ordem) ─────────────────────────
     * alpha = 1 - exp(-2π × fc × dt)
     * fc = ffb_filter_hz, dt = 0.01 s.
     * filter_hz == 0 → filtro desabilitado (alpha = 1.0, sem latência).
     *
     * Valores práticos:
     *   10 Hz → alpha ≈ 0.47  (suave, ~95 ms lag)
     *   30 Hz → alpha ≈ 0.85  (equilíbrio, ~30 ms lag)
     *   60 Hz → alpha ≈ 0.98  (quase transparente, ~16 ms lag)
     *  100 Hz → alpha ≈ 0.998 (transparente)                                */
    {
        static float torque_filtered = 0.0f;
        float alpha;
        if (g_config.ffb_filter_hz > 0.0f) {
            /* Pré-calcular: 2π × fc × dt ≈ 0.06283 × fc */
            float x = 6.2832f * g_config.ffb_filter_hz * FFB_DT_S;
            /* Aproximação de exp(-x) via série de Taylor para x < 4 (fc < 64 Hz)
             * ou clamp para x ≥ 4 → exp(-4) ≈ 0.018 → alpha ≈ 0.982        */
            float ex;
            if (x >= 4.0f) {
                ex = 0.0183f;   /* exp(-4) */
            } else {
                /* exp(-x) ≈ 1 - x + x²/2 - x³/6 + x⁴/24  (erro < 0.1% para x<4) */
                float x2 = x * x;
                ex = 1.0f - x + x2 * 0.5f - x2 * x * 0.16667f + x2 * x2 * 0.04167f;
            }
            alpha = 1.0f - ex;
            if (alpha < 0.0f) alpha = 0.0f;
            if (alpha > 1.0f) alpha = 1.0f;
        } else {
            alpha = 1.0f;   /* sem filtro */
        }
        torque_filtered += alpha * (torque - torque_filtered);
        torque = torque_filtered;
    }

    /* Clamp final de segurança */
    return fclamp(torque, -max_t, max_t);
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
