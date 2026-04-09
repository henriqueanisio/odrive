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
#define CFG_MOTOR_CURRENT_LIM               200U
#define CFG_MOTOR_TORQUE_CONSTANT           201U
#define CFG_MOTOR_POLE_PAIRS                202U
#define CFG_MOTOR_TYPE                      203U  /* 0=HIGH_CURRENT, 1=ACIM, 2=GIMBAL */
#define CFG_MOTOR_BANDWIDTH                 204U  /* current_control_bandwidth (Hz)    */
#define CFG_MOTOR_CALIB_CURRENT             205U  /* calibration_current (A)           */
#define CFG_MOTOR_CALIB_VOLTAGE             206U  /* resistance_calib_max_voltage (V)  */
#define CFG_MOTOR_PRE_CALIBRATED            207U  /* pre_calibrated: 0/1               */

/* Encoder */
#define CFG_ENCODER_CPR                     301U
#define CFG_ENCODER_DIRECTION               302U  /* +1 / -1                           */
#define CFG_ENCODER_OFFSET                  303U  /* electrical phase offset (rad)     */
#define CFG_ENCODER_MODE                    304U  /* 0=INCREMENTAL,1=HALL,257=AMS_SPI  */
#define CFG_ENCODER_BANDWIDTH               305U  /* bandwidth (Hz)                    */
#define CFG_ENCODER_ABS_SPI_CS_GPIO         306U  /* abs_spi_cs_gpio_pin               */
#define CFG_ENCODER_PRE_CALIBRATED          307U  /* pre_calibrated: 0/1               */

/* Controller */
#define CFG_CTRL_POS_GAIN                   410U
#define CFG_CTRL_VEL_GAIN                   411U
#define CFG_CTRL_VEL_INTEGRATOR_GAIN        412U
#define CFG_CTRL_VEL_LIMIT                  413U
#define CFG_CTRL_CONTROL_MODE               414U  /* 0=VOLTAGE,1=TORQUE,2=VEL,3=POS   */

/* Limits */
#define CFG_LIM_VBUS_UNDERVOLTAGE           500U
#define CFG_LIM_VBUS_OVERVOLTAGE            501U
#define CFG_LIM_BRAKE_RESISTOR_ENABLE       502U  /* 0/1                               */
#define CFG_LIM_BRAKE_RESISTANCE            503U  /* Ohms                              */

/* ── Config struct ─────────────────────────────────────────────────────────── */
typedef struct {
    /* Motor */
    float   current_lim;                  /* A       peak phase current             */
    float   torque_constant;              /* Nm/A    motor Kt                        */
    int32_t pole_pairs;
    int32_t motor_type;                   /* Motor::MotorType enum                  */
    float   current_control_bandwidth;   /* Hz                                      */
    float   calibration_current;         /* A                                       */
    float   resistance_calib_max_voltage;/* V                                       */
    int32_t motor_pre_calibrated;        /* bool: 0/1                               */

    /* Encoder */
    int32_t encoder_cpr;
    int32_t encoder_direction;           /* +1/-1                                   */
    float   encoder_offset;              /* rad                                     */
    int32_t encoder_mode;               /* Encoder::Mode enum (257 = ABS_SPI_AMS)  */
    float   encoder_bandwidth;          /* Hz                                       */
    int32_t abs_spi_cs_gpio_pin;
    int32_t encoder_pre_calibrated;     /* bool: 0/1                                */

    /* Controller */
    float   pos_gain;
    float   vel_gain;
    float   vel_integrator_gain;
    float   vel_limit;
    int32_t control_mode;               /* ControlMode enum: 0-3                    */

    /* Limits */
    float   vbus_undervoltage;
    float   vbus_overvoltage;
    int32_t enable_brake_resistor;      /* bool: 0/1                                */
    float   brake_resistance;          /* Ohms                                      */
} ODriveConfig_t;

/* Factory defaults */
#define CONFIG_DEFAULT {                                \
    .current_lim                   = 10.0f,           \
    .torque_constant               = 0.04f,           \
    .pole_pairs                    = 7,               \
    .motor_type                    = 0,               \
    .current_control_bandwidth     = 100.0f,          \
    .calibration_current           = 10.0f,           \
    .resistance_calib_max_voltage  = 2.0f,            \
    .motor_pre_calibrated          = 0,               \
    .encoder_cpr                   = 8192,            \
    .encoder_direction             = 1,               \
    .encoder_offset                = 0.0f,            \
    .encoder_mode                  = 0,               \
    .encoder_bandwidth             = 100.0f,          \
    .abs_spi_cs_gpio_pin           = 7,               \
    .encoder_pre_calibrated        = 0,               \
    .pos_gain                      = 20.0f,           \
    .vel_gain                      = 0.16f,           \
    .vel_integrator_gain           = 0.32f,           \
    .vel_limit                     = 20000.0f,        \
    .control_mode                  = 3,               \
    .vbus_undervoltage             = 8.0f,            \
    .vbus_overvoltage              = 56.0f,           \
    .enable_brake_resistor         = 0,               \
    .brake_resistance              = 2.0f,            \
}

extern ODriveConfig_t g_config;

void config_init(void);
bool config_set(uint16_t param_id, float value);
bool config_get(uint16_t param_id, float *value_out);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
