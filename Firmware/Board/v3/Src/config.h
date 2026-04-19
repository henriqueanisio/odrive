#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── FFB engine constants (used by ffb_engine.c / ffb_forces.c) ────────────── */
#define MAX_EFFECTS       40
#define MEMORY_POOL_SIZE  0x3fff
#define DURATION_INF      0xffff
#define FFB_DEAD_ZONE     0x00ff

#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

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
#define CFG_MOTOR_PHASE_RESISTANCE          208U  /* measured phase resistance (Ω)     */
#define CFG_MOTOR_PHASE_INDUCTANCE          209U  /* measured phase inductance (H)     */

/* Encoder */
#define CFG_ENCODER_CPR                     301U
#define CFG_ENCODER_DIRECTION               302U  /* +1 / -1                           */
#define CFG_ENCODER_OFFSET                  303U  /* electrical phase offset (rad)     */
#define CFG_ENCODER_MODE                    304U  /* 0=INCREMENTAL,1=HALL,257=AMS_SPI  */
#define CFG_ENCODER_BANDWIDTH               305U  /* bandwidth (Hz)                    */
#define CFG_ENCODER_ABS_SPI_CS_GPIO         306U  /* abs_spi_cs_gpio_pin               */
#define CFG_ENCODER_PRE_CALIBRATED          307U  /* pre_calibrated: 0/1               */
#define CFG_ENCODER_USE_INDEX               308U  /* use Z/index channel: 0/1          */
#define CFG_ENCODER_PIN_A                   309U
#define CFG_ENCODER_PIN_B                   310U
#define CFG_ENCODER_GPIO_PULL               311U
#define CFG_ENCODER_FILTER                  312U

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

/* Steering */
#define CFG_STEERING_MAX_LOCK               510U  /* total degrees of wheel travel     */

/* Force Feedback */
#define CFG_FFB_MAX_TORQUE                  600U  /* Nm — peak output torque           */
#define CFG_FFB_GAIN                        601U  /* 0.0–1.0 — global FFB scale        */
#define CFG_FFB_DAMPING                     602U  /* 0.0–1.0 — velocity damping coeff  */
#define CFG_FFB_FRICTION                    603U  /* 0.0–1.0 — Coulomb friction coeff  */
#define CFG_FFB_INERTIA                     604U  /* 0.0–1.0 — inertia (mass sim)      */
#define CFG_FFB_SPRING                      605U  /* 0.0–1.0 — center spring coeff     */
#define CFG_FFB_SLEW_RATE                   606U  /* Nm/ms — output slew rate (0=off)  */
#define CFG_FFB_FILTER_HZ                   607U  /* Hz — output low-pass (0=off)      */
#define CFG_FFB_MIN_FORCE                   608U  /* 0.0–1.0 — dead-zone fraction      */
#define CFG_FFB_ENDSTOP_STRENGTH            609U  /* 0.0–1.0 — endstop counterforce    */
#define CFG_FFB_ENDSTOP_RANGE               610U  /* 0.0–0.5 — fade zone (frac of lock)*/

/* Force Feedback — Extended (professional grade) */
#define CFG_FFB_INPUT_FILTER_HZ             611U  /* Hz — pre-DI input filter (0=off)  */
#define CFG_FFB_CENTER_BOOST_STR            612U  /* 0.0–1.0 — center boost strength   */
#define CFG_FFB_CENTER_BOOST_WIDTH          613U  /* turns — center boost half-width   */
#define CFG_FFB_STATIC_FRICTION             614U  /* 0.0–1.0 — static (breakaway) fric */
#define CFG_FFB_STRIBECK_VEL                615U  /* turns/s — Stribeck velocity        */
#define CFG_FFB_INERTIA_FILTER_HZ           616U  /* Hz — accel derivative filter       */
#define CFG_FFB_ENDSTOP_EXP                 617U  /* 1–4 — endstop fade curve exponent  */
#define CFG_FFB_THERMAL_TAU                 618U  /* s — I²t thermal time constant      */
#define CFG_FFB_THERMAL_MIN                 619U  /* 0.0–1.0 — min scale during derate  */
#define CFG_FFB_LUT_EN                      620U  /* 0/1 — enable torque LUT            */
#define CFG_FFB_LUT_P0                      621U  /* LUT point 0 (|t|=0.000) [0..1]    */
#define CFG_FFB_LUT_P1                      622U  /* LUT point 1 (|t|=0.143) [0..1]    */
#define CFG_FFB_LUT_P2                      623U  /* LUT point 2 (|t|=0.286) [0..1]    */
#define CFG_FFB_LUT_P3                      624U  /* LUT point 3 (|t|=0.429) [0..1]    */
#define CFG_FFB_LUT_P4                      625U  /* LUT point 4 (|t|=0.571) [0..1]    */
#define CFG_FFB_LUT_P5                      626U  /* LUT point 5 (|t|=0.714) [0..1]    */
#define CFG_FFB_LUT_P6                      627U  /* LUT point 6 (|t|=0.857) [0..1]    */
#define CFG_FFB_LUT_P7                      628U  /* LUT point 7 (|t|=1.000) [0..1]    */

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
    float   phase_resistance;           /* Ω — measured by motor calibration        */
    float   phase_inductance;           /* H — measured by motor calibration        */

    /* Encoder */
    int32_t encoder_cpr;
    int32_t encoder_direction;           /* +1/-1                                   */
    float   encoder_offset;              /* rad                                     */
    int32_t encoder_mode;               /* Encoder::Mode enum (257 = ABS_SPI_AMS)  */
    float   encoder_bandwidth;          /* Hz                                       */
    int32_t abs_spi_cs_gpio_pin;
    int32_t encoder_pre_calibrated;     /* bool: 0/1                                */
    int32_t encoder_use_index;          /* bool: 0/1 — enable Z/index channel       */
    int32_t encoder_pin_a;
    int32_t encoder_pin_b;
    int32_t encoder_gpio_pull;
    int32_t encoder_filter;
    
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

    /* Steering */
    float   steering_max_lock;         /* total degrees of wheel travel (e.g. 900)  */

    /* Force Feedback */
    float   ffb_max_torque;            /* Nm peak — clamps all FFB output           */
    float   ffb_gain;                  /* 0.0–1.0 global scale applied after effects */
    float   ffb_damping;               /* 0.0–1.0 — velocity-proportional damping   */
    float   ffb_friction;              /* 0.0–1.0 — Coulomb friction coefficient    */
    float   ffb_inertia;               /* 0.0–1.0 — rotational inertia (mass sim)   */
    float   ffb_spring;                /* 0.0–1.0 — always-on center spring         */
    float   ffb_slew_rate;             /* Nm/ms — torque slew rate limit (0=off)    */
    float   ffb_filter_hz;             /* Hz — output low-pass filter (0=off)       */
    float   ffb_min_force;             /* 0.0–1.0 — minimum force dead-zone         */
    float   ffb_endstop_strength;      /* 0.0–1.0 — endstop counterforce fraction   */
    float   ffb_endstop_range;         /* 0.0–0.5 — fade zone as fraction of lock   */

    /* Force Feedback — Extended (professional grade) */
    float   ffb_input_filter_hz;       /* Hz — IIR filter on DI effects before gain  */
    float   ffb_center_boost_str;      /* 0.0–1.0 — extra spring force near center   */
    float   ffb_center_boost_width;    /* 0.0–1.0 normalized — fraction of lock half-range */
    float   ffb_static_friction;       /* 0.0–1.0 — static breakaway friction coeff  */
    float   ffb_stribeck_vel;          /* turns/s — vel where static→dynamic transits*/
    float   ffb_inertia_filter_hz;     /* Hz — low-pass filter on accel derivative   */
    int32_t ffb_endstop_exp;           /* 1–4 — endstop curve exponent (2=quadratic) */
    float   ffb_thermal_tau;           /* s — I²t thermal model time constant        */
    float   ffb_thermal_min;           /* 0.0–1.0 — minimum scale during derating    */
    int32_t ffb_lut_enabled;           /* 0/1 — enable torque linearization LUT      */
    float   ffb_lut[8];                /* LUT correction for |torque| ∈ [0..1]       */
} ODriveConfig_t;

/* Factory defaults */
#define CONFIG_DEFAULT {                                \
    .current_lim                   = 25.0f,             \
    .torque_constant               = 0.5f,              \
    .pole_pairs                    = 15,                \
    .motor_type                    = 0,                 \
    .current_control_bandwidth     = 100.0f,            \
    .calibration_current           = 4.0f,              \
    .resistance_calib_max_voltage  = 2.0f,              \
    .motor_pre_calibrated          = 0,                 \
    .phase_resistance              = 0.0f,              \
    .phase_inductance              = 0.0f,              \
    .encoder_cpr                   = 2400,              \
    .encoder_direction             = 1,                 \
    .encoder_offset                = 0.0f,              \
    .encoder_mode                  = 0,                 \
    .encoder_bandwidth             = 200.0f,            \
    .abs_spi_cs_gpio_pin           = 7,                 \
    .encoder_pre_calibrated        = 0,                 \
    .encoder_use_index             = 0,                 \
    .pos_gain                      = 0.0f,              \
    .vel_gain                      = 0.0f,              \
    .vel_integrator_gain           = 0.0f,              \
    .vel_limit                     = 20000.0f,          \
    .control_mode                  = 1,                 \
    .vbus_undervoltage             = 8.0f,              \
    .vbus_overvoltage              = 24.0f,             \
    .enable_brake_resistor         = 1,                 \
    .brake_resistance              = 12.0f,             \
    .steering_max_lock             = 900.0f,            \
    .ffb_max_torque                = 3.0f,              \
    .ffb_gain                      = 1.0f,              \
    .ffb_min_force                 = 0.02f,             \
    .ffb_damping                   = 0.08f,             \
    .ffb_friction                  = 0.015f,            \
    .ffb_inertia                   = 0.025f,            \
    .ffb_spring                    = 0.05f,             \
    .ffb_slew_rate                 = 0.08f,             \
    .ffb_filter_hz                 = 80.0f,             \
    .ffb_endstop_strength          = 0.7f,              \
    .ffb_endstop_range             = 0.08f,             \
    .ffb_input_filter_hz           = 120.0f,            \
    .ffb_center_boost_str          = 0.1f,              \
    .ffb_center_boost_width        = 0.12f,             \
    .ffb_static_friction           = 0.02f,             \
    .ffb_stribeck_vel              = 0.03f,             \
    .ffb_inertia_filter_hz         = 8.0f,              \
    .ffb_endstop_exp               = 2.5f,              \
    .ffb_thermal_tau               = 20.0f,             \
    .ffb_thermal_min               = 0.5f,              \
    .ffb_lut_enabled               = 0,                 \
    .ffb_lut = {0.0000f, 0.12f, 0.26f, 0.40f,           \
                0.55f, 0.70f, 0.85f, 1.0000f},          \
}

extern ODriveConfig_t g_config;

void config_init(void);
bool config_set(uint16_t param_id, float value);
bool config_get(uint16_t param_id, float *value_out);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
