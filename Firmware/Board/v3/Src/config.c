#include "config.h"
#include <string.h>

/* Global config — starts with factory defaults; overwritten by flash_load. */
ODriveConfig_t g_config = CONFIG_DEFAULT;

void config_init(void)
{
    ODriveConfig_t defaults = CONFIG_DEFAULT;
    g_config = defaults;
}

/* ── config_set ──────────────────────────────────────────────────────────────
 * Write one parameter with basic range validation.
 * Values outside the accepted range are silently rejected (returns false).
 * ─────────────────────────────────────────────────────────────────────────── */
bool config_set(uint16_t param_id, float value)
{
    switch (param_id) {

        /* ── Motor (200–299) ── */

        case CFG_MOTOR_CURRENT_LIM:
            if (value < 0.1f || value > 120.0f) return false;
            g_config.current_lim = value;
            return true;

        case CFG_MOTOR_TORQUE_CONSTANT:
            if (value <= 0.0f || value > 10.0f) return false;
            g_config.torque_constant = value;
            return true;

        case CFG_MOTOR_POLE_PAIRS: {
            int32_t pp = (int32_t)value;
            if (pp < 1 || pp > 100) return false;
            g_config.pole_pairs = pp;
            return true;
        }

        /* ── Encoder (300–399) ── */

        case CFG_ENCODER_CPR: {
            int32_t cpr = (int32_t)value;
            if (cpr < 4) return false;
            g_config.encoder_cpr = cpr;
            return true;
        }

        case CFG_ENCODER_DIRECTION:
            g_config.encoder_direction = (value >= 0.0f) ? 1 : -1;
            return true;

        case CFG_ENCODER_OFFSET:
            /* offset lives in [-pi, +pi] after calibration */
            g_config.encoder_offset = value;
            return true;

        /* ── Controller (400–499) ── */

        case CFG_CTRL_POS_GAIN:
            if (value < 0.0f) return false;
            g_config.pos_gain = value;
            return true;

        case CFG_CTRL_VEL_GAIN:
            if (value < 0.0f) return false;
            g_config.vel_gain = value;
            return true;

        case CFG_CTRL_VEL_INTEGRATOR_GAIN:
            if (value < 0.0f) return false;
            g_config.vel_integrator_gain = value;
            return true;

        case CFG_CTRL_VEL_LIMIT:
            if (value <= 0.0f) return false;
            g_config.vel_limit = value;
            return true;

        /* ── Limits (500–599) ── */

        case CFG_LIM_VBUS_UNDERVOLTAGE:
            if (value < 0.0f) return false;
            g_config.vbus_undervoltage = value;
            return true;

        case CFG_LIM_VBUS_OVERVOLTAGE:
            if (value <= g_config.vbus_undervoltage) return false;
            g_config.vbus_overvoltage = value;
            return true;

        default:
            return false;
    }
}

/* ── config_get ──────────────────────────────────────────────────────────────
 * Integer parameters are cast to float for uniform transport.
 * ─────────────────────────────────────────────────────────────────────────── */
bool config_get(uint16_t param_id, float *value_out)
{
    if (!value_out) return false;

    switch (param_id) {
        case CFG_MOTOR_CURRENT_LIM:
            *value_out = g_config.current_lim;             return true;
        case CFG_MOTOR_TORQUE_CONSTANT:
            *value_out = g_config.torque_constant;         return true;
        case CFG_MOTOR_POLE_PAIRS:
            *value_out = (float)g_config.pole_pairs;       return true;
        case CFG_ENCODER_CPR:
            *value_out = (float)g_config.encoder_cpr;      return true;
        case CFG_ENCODER_DIRECTION:
            *value_out = (float)g_config.encoder_direction;return true;
        case CFG_ENCODER_OFFSET:
            *value_out = g_config.encoder_offset;          return true;
        case CFG_CTRL_POS_GAIN:
            *value_out = g_config.pos_gain;                return true;
        case CFG_CTRL_VEL_GAIN:
            *value_out = g_config.vel_gain;                return true;
        case CFG_CTRL_VEL_INTEGRATOR_GAIN:
            *value_out = g_config.vel_integrator_gain;     return true;
        case CFG_CTRL_VEL_LIMIT:
            *value_out = g_config.vel_limit;               return true;
        case CFG_LIM_VBUS_UNDERVOLTAGE:
            *value_out = g_config.vbus_undervoltage;       return true;
        case CFG_LIM_VBUS_OVERVOLTAGE:
            *value_out = g_config.vbus_overvoltage;        return true;
        default:
            return false;
    }
}
