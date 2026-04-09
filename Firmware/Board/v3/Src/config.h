#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Parameter ID ranges ──────────────────────────────────────────────────────
 *   Motor      200–299
 *   Encoder    300–399
 *   Controller 400–499
 *   Limits     500–599
 * ─────────────────────────────────────────────────────────────────────────── */

/* Motor */
#define CFG_MOTOR_CURRENT_LIM           200U
#define CFG_MOTOR_TORQUE_CONSTANT       201U
#define CFG_MOTOR_POLE_PAIRS            202U

/* Encoder */
#define CFG_ENCODER_CPR                 301U
#define CFG_ENCODER_DIRECTION           302U
#define CFG_ENCODER_OFFSET              303U

/* Controller */
#define CFG_CTRL_POS_GAIN               410U
#define CFG_CTRL_VEL_GAIN               411U
#define CFG_CTRL_VEL_INTEGRATOR_GAIN    412U
#define CFG_CTRL_VEL_LIMIT              413U

/* Limits */
#define CFG_LIM_VBUS_UNDERVOLTAGE       500U
#define CFG_LIM_VBUS_OVERVOLTAGE        501U

/* ── Config struct ─────────────────────────────────────────────────────────── */
typedef struct {
    /* Motor */
    float   current_lim;            /* A       — peak phase current            */
    float   torque_constant;        /* Nm/A    — motor Kt                      */
    int32_t pole_pairs;             /* —       — number of pole pairs          */

    /* Encoder */
    int32_t encoder_cpr;            /* cnt/rev — counts per revolution         */
    int32_t encoder_direction;      /* +1/-1   — positive direction            */
    float   encoder_offset;         /* rad     — electrical phase offset       */

    /* Controller */
    float   pos_gain;               /* (cnt/s)/cnt — position P gain          */
    float   vel_gain;               /* A/(cnt/s)   — velocity P gain          */
    float   vel_integrator_gain;    /* A/cnt       — velocity integrator gain */
    float   vel_limit;              /* cnt/s       — velocity limit           */

    /* Limits */
    float   vbus_undervoltage;      /* V   — undervoltage trip                 */
    float   vbus_overvoltage;       /* V   — overvoltage trip                  */
} ODriveConfig_t;

/* Factory defaults */
#define CONFIG_DEFAULT {                \
    .current_lim          = 10.0f,     \
    .torque_constant      = 0.04f,     \
    .pole_pairs           = 7,         \
    .encoder_cpr          = 8192,      \
    .encoder_direction    = 1,         \
    .encoder_offset       = 0.0f,      \
    .pos_gain             = 20.0f,     \
    .vel_gain             = 0.16f,     \
    .vel_integrator_gain  = 0.32f,     \
    .vel_limit            = 20000.0f,  \
    .vbus_undervoltage    = 8.0f,      \
    .vbus_overvoltage     = 56.0f,     \
}

/* ── Global instance (defined in config.c) ────────────────────────────────── */
extern ODriveConfig_t g_config;

/* ── API ───────────────────────────────────────────────────────────────────── */

/* Load factory defaults into g_config */
void config_init(void);

/* Write a parameter value.  Returns false if param_id is unknown or value
 * is out of range.  Thread-safe: float writes on 32-bit ARM are atomic. */
bool config_set(uint16_t param_id, float value);

/* Read a parameter value into *value_out.
 * Returns false if param_id is unknown.                                    */
bool config_get(uint16_t param_id, float *value_out);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
