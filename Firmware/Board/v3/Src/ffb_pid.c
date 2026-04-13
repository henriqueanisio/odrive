#include "ffb_pid.h"
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
float ffb_compute_torque(float pos_turns, float vel_turns_s)
{
    if (!s.actuators_enabled || s.paused) return 0.0f;

    float torque    = 0.0f;
    float max_t     = g_config.ffb_max_torque;
    float gain_norm = (float)s.device_gain / 255.0f;
    uint32_t now    = HAL_GetTick();

    for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
        FfbEffect_t *e = &s.effects[i];
        if (!e->active || e->type == FFB_ET_NONE) continue;

        /* duração */
        if (e->duration_ms != 0xFFFFU) {
            if ((now - e->start_tick) >= (uint32_t)e->duration_ms) {
                e->active = false;
                continue;
            }
        }

        float effect_gain = (float)e->gain / 255.0f;

        switch (e->type) {

        case FFB_ET_CONSTANT: {
            float t = ((float)e->magnitude / 10000.0f) * max_t;
            torque += t * effect_gain;
            break;
        }

        case FFB_ET_SPRING: {
            float cp_turns = ((float)e->cp_offset / 10000.0f);
            float err      = pos_turns - cp_turns;

            float db = (float)e->dead_band / 10000.0f;
            if (err > -db && err < db) break;

            float coeff = (err >= 0.0f)
                ? ((float)e->pos_coeff / 10000.0f)
                : ((float)e->neg_coeff / 10000.0f);

            /* 🔥 NÃO-LINEAR (Simucube feel) */
            float nonlinear = err * fabsf(err);

            float t = -coeff * nonlinear * max_t;

            float sat = ((float)e->pos_sat / 10000.0f) * max_t;
            t = fclamp(t, -sat, sat);

            torque += t * effect_gain;
            break;
        }

        case FFB_ET_DAMPER: {
            float coeff = (float)e->pos_coeff / 10000.0f;
            float t     = -coeff * vel_turns_s * max_t;
            torque += t * effect_gain;
            break;
        }

        default:
            break;
        }
    }

    /* 🔥 DAMPER GLOBAL (sempre ativo) */
    float global_damper = 0.05f;
    torque -= vel_turns_s * global_damper * max_t;

    /* 🔥 GAIN GLOBAL */
    torque *= gain_norm * g_config.ffb_gain;

    /* 🔥 RATE LIMIT (anti spike) */
    static float last_torque = 0.0f;
    float max_delta = max_t * 0.15f;

    float delta = torque - last_torque;

    if (delta >  max_delta) delta =  max_delta;
    if (delta < -max_delta) delta = -max_delta;

    torque = last_torque + delta;
    last_torque = torque;

    /* 🔥 SMOOTHING */
    static float torque_filtered = 0.0f;
    float alpha = 0.2f;

    torque_filtered += alpha * (torque - torque_filtered);

    return fclamp(torque_filtered, -max_t, max_t);
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
