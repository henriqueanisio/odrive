#include "config.h"
#include <string.h>

ODriveConfig_t g_config = CONFIG_DEFAULT;

void config_init(void)
{
    ODriveConfig_t defaults = CONFIG_DEFAULT;
    g_config = defaults;
}

bool config_set(uint16_t param_id, float value)
{
    switch (param_id) {

        /* ── Motor (200–299) ── */
        case CFG_MOTOR_CURRENT_LIM:
            if (value < 0.1f || value > 120.0f) return false;
            g_config.current_lim = value;                            return true;

        case CFG_MOTOR_TORQUE_CONSTANT:
            if (value <= 0.0f || value > 10.0f) return false;
            g_config.torque_constant = value;                        return true;

        case CFG_MOTOR_POLE_PAIRS: {
            int32_t pp = (int32_t)value;
            if (pp < 1 || pp > 100) return false;
            g_config.pole_pairs = pp;                                return true;
        }

        case CFG_MOTOR_TYPE: {
            int32_t t = (int32_t)value;
            if (t < 0 || t > 2) return false;
            g_config.motor_type = t;                                 return true;
        }

        case CFG_MOTOR_BANDWIDTH:
            if (value <= 0.0f || value > 10000.0f) return false;
            g_config.current_control_bandwidth = value;              return true;

        case CFG_MOTOR_CALIB_CURRENT:
            if (value <= 0.0f || value > 120.0f) return false;
            g_config.calibration_current = value;                    return true;

        case CFG_MOTOR_CALIB_VOLTAGE:
            if (value <= 0.0f || value > 30.0f) return false;
            g_config.resistance_calib_max_voltage = value;           return true;

        case CFG_MOTOR_PRE_CALIBRATED:
            g_config.motor_pre_calibrated = (value != 0.0f) ? 1 : 0; return true;

        case CFG_MOTOR_PHASE_RESISTANCE:
            if (value < 0.0f || value > 10.0f) return false;
            g_config.phase_resistance = value;                          return true;

        case CFG_MOTOR_PHASE_INDUCTANCE:
            if (value < 0.0f || value > 0.01f) return false;
            g_config.phase_inductance = value;                          return true;

        /* ── Encoder (300–399) ── */
        case CFG_ENCODER_CPR: {
            int32_t cpr = (int32_t)value;
            if (cpr < 4) return false;
            g_config.encoder_cpr = cpr;                              return true;
        }

        case CFG_ENCODER_DIRECTION:
            g_config.encoder_direction = (value >= 0.0f) ? 1 : -1;  return true;

        case CFG_ENCODER_OFFSET:
            g_config.encoder_offset = value;                         return true;

        case CFG_ENCODER_MODE:
            g_config.encoder_mode = (int32_t)value;                  return true;

        case CFG_ENCODER_BANDWIDTH:
            if (value <= 0.0f || value > 10000.0f) return false;
            g_config.encoder_bandwidth = value;                      return true;

        case CFG_ENCODER_ABS_SPI_CS_GPIO: {
            int32_t pin = (int32_t)value;
            if (pin < 0 || pin > 15) return false;
            g_config.abs_spi_cs_gpio_pin = pin;                      return true;
        }

        case CFG_ENCODER_PRE_CALIBRATED:
            g_config.encoder_pre_calibrated = (value != 0.0f) ? 1 : 0; return true;

        case CFG_ENCODER_USE_INDEX:
            g_config.encoder_use_index = (value != 0.0f) ? 1 : 0;      return true;

        /* ── Controller (400–499) ── */
        case CFG_CTRL_POS_GAIN:
            if (value < 0.0f) return false;
            g_config.pos_gain = value;                               return true;

        case CFG_CTRL_VEL_GAIN:
            if (value < 0.0f) return false;
            g_config.vel_gain = value;                               return true;

        case CFG_CTRL_VEL_INTEGRATOR_GAIN:
            if (value < 0.0f) return false;
            g_config.vel_integrator_gain = value;                    return true;

        case CFG_CTRL_VEL_LIMIT:
            if (value <= 0.0f) return false;
            g_config.vel_limit = value;                              return true;

        case CFG_CTRL_CONTROL_MODE: {
            int32_t m = (int32_t)value;
            if (m < 0 || m > 3) return false;
            g_config.control_mode = m;                               return true;
        }

        /* ── Limits (500–599) ── */
        case CFG_LIM_VBUS_UNDERVOLTAGE:
            if (value < 0.0f) return false;
            g_config.vbus_undervoltage = value;                      return true;

        case CFG_LIM_VBUS_OVERVOLTAGE:
            if (value <= g_config.vbus_undervoltage) return false;
            g_config.vbus_overvoltage = value;                       return true;

        case CFG_LIM_BRAKE_RESISTOR_ENABLE:
            g_config.enable_brake_resistor = (value != 0.0f) ? 1 : 0; return true;

        case CFG_LIM_BRAKE_RESISTANCE:
            if (value <= 0.0f || value > 100.0f) return false;
            g_config.brake_resistance = value;                       return true;

        /* ── Steering (510) ── */
        case CFG_STEERING_MAX_LOCK:
            if (value < 90.0f || value > 2160.0f) return false;
            g_config.steering_max_lock = value;                      return true;

        default:
            return false;
    }
}

bool config_get(uint16_t param_id, float *value_out)
{
    if (!value_out) return false;

    switch (param_id) {
        case CFG_MOTOR_CURRENT_LIM:
            *value_out = g_config.current_lim;                        return true;
        case CFG_MOTOR_TORQUE_CONSTANT:
            *value_out = g_config.torque_constant;                    return true;
        case CFG_MOTOR_POLE_PAIRS:
            *value_out = (float)g_config.pole_pairs;                  return true;
        case CFG_MOTOR_TYPE:
            *value_out = (float)g_config.motor_type;                  return true;
        case CFG_MOTOR_BANDWIDTH:
            *value_out = g_config.current_control_bandwidth;          return true;
        case CFG_MOTOR_CALIB_CURRENT:
            *value_out = g_config.calibration_current;                return true;
        case CFG_MOTOR_CALIB_VOLTAGE:
            *value_out = g_config.resistance_calib_max_voltage;       return true;
        case CFG_MOTOR_PRE_CALIBRATED:
            *value_out = (float)g_config.motor_pre_calibrated;        return true;
        case CFG_MOTOR_PHASE_RESISTANCE:
            *value_out = g_config.phase_resistance;                   return true;
        case CFG_MOTOR_PHASE_INDUCTANCE:
            *value_out = g_config.phase_inductance;                   return true;

        case CFG_ENCODER_CPR:
            *value_out = (float)g_config.encoder_cpr;                 return true;
        case CFG_ENCODER_DIRECTION:
            *value_out = (float)g_config.encoder_direction;           return true;
        case CFG_ENCODER_OFFSET:
            *value_out = g_config.encoder_offset;                     return true;
        case CFG_ENCODER_MODE:
            *value_out = (float)g_config.encoder_mode;                return true;
        case CFG_ENCODER_BANDWIDTH:
            *value_out = g_config.encoder_bandwidth;                  return true;
        case CFG_ENCODER_ABS_SPI_CS_GPIO:
            *value_out = (float)g_config.abs_spi_cs_gpio_pin;         return true;
        case CFG_ENCODER_PRE_CALIBRATED:
            *value_out = (float)g_config.encoder_pre_calibrated;      return true;
        case CFG_ENCODER_USE_INDEX:
            *value_out = (float)g_config.encoder_use_index;           return true;

        case CFG_CTRL_POS_GAIN:
            *value_out = g_config.pos_gain;                           return true;
        case CFG_CTRL_VEL_GAIN:
            *value_out = g_config.vel_gain;                           return true;
        case CFG_CTRL_VEL_INTEGRATOR_GAIN:
            *value_out = g_config.vel_integrator_gain;                return true;
        case CFG_CTRL_VEL_LIMIT:
            *value_out = g_config.vel_limit;                          return true;
        case CFG_CTRL_CONTROL_MODE:
            *value_out = (float)g_config.control_mode;                return true;

        case CFG_LIM_VBUS_UNDERVOLTAGE:
            *value_out = g_config.vbus_undervoltage;                  return true;
        case CFG_LIM_VBUS_OVERVOLTAGE:
            *value_out = g_config.vbus_overvoltage;                   return true;
        case CFG_LIM_BRAKE_RESISTOR_ENABLE:
            *value_out = (float)g_config.enable_brake_resistor;       return true;
        case CFG_LIM_BRAKE_RESISTANCE:
            *value_out = g_config.brake_resistance;                   return true;

        case CFG_STEERING_MAX_LOCK:
            *value_out = g_config.steering_max_lock;                  return true;

        default:
            return false;
    }
}
