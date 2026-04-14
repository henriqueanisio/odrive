#ifndef FFB_PID_H
#define FFB_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Effect slots ────────────────────────────────────────────────────────────
 * We expose FFB_MAX_EFFECTS slots to the host (effect_block_index 1..N).
 * A single-axis wheel rarely needs more than 4 simultaneous effects.
 * ─────────────────────────────────────────────────────────────────────────── */
#define FFB_MAX_EFFECTS  4U

/* ── Effect types (match selector index in HID descriptor, 1-based) ─────────
 *   1 = ET_CONSTANT_FORCE  (usage 0x26)
 *   2 = ET_SPRING          (usage 0x40)
 *   3 = ET_DAMPER          (usage 0x41)
 * ─────────────────────────────────────────────────────────────────────────── */
#define FFB_ET_NONE      0U
#define FFB_ET_CONSTANT  1U
#define FFB_ET_SPRING    2U
#define FFB_ET_DAMPER    3U

/* ── HID PID report IDs (host → device, Feature/Output via EP0 SET_REPORT) ──
 *   IDs 1–4 reserved for existing joystick/telemetry/command/config reports.
 * ─────────────────────────────────────────────────────────────────────────── */
#define FFB_REPORT_SET_EFFECT          0x05U
#define FFB_REPORT_SET_CONDITION       0x07U
#define FFB_REPORT_SET_CONSTANT_FORCE  0x08U
#define FFB_REPORT_EFFECT_OPERATION    0x0BU
#define FFB_REPORT_DEVICE_CONTROL      0x0CU
#define FFB_REPORT_DEVICE_GAIN         0x0DU

/* HID PID report IDs (device → host) */
#define FFB_REPORT_PID_STATE           0x0EU  /* IN  — polled via joystick send */
#define FFB_REPORT_PID_BLOCK_LOAD      0x0FU  /* Feature GET_REPORT              */
#define FFB_REPORT_PID_POOL            0x10U  /* Feature GET_REPORT              */

/* ── Effect Operation op-codes ───────────────────────────────────────────── */
#define FFB_OP_START       1U
#define FFB_OP_START_SOLO  2U
#define FFB_OP_STOP        3U

/* ── Device Control op-codes ─────────────────────────────────────────────── */
#define FFB_DC_ENABLE_ACTUATORS   1U
#define FFB_DC_DISABLE_ACTUATORS  2U
#define FFB_DC_STOP_ALL_EFFECTS   3U
#define FFB_DC_DEVICE_RESET       4U
#define FFB_DC_DEVICE_PAUSE       5U
#define FFB_DC_DEVICE_CONTINUE    6U

/* ── HID PID packed report structures (host → device) ───────────────────────
 * Byte layout must match the HID report descriptor exactly.
 * ─────────────────────────────────────────────────────────────────────────── */

/* Report 0x05 — Set Effect (11 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  effect_block_index;   /* 1-based slot                              */
    uint8_t  effect_type;          /* FFB_ET_xxx selector (1=const,2=spr,3=dmp) */
    uint16_t duration;             /* ms; 0xFFFF = infinite                     */
    uint16_t trigger_repeat;       /* ms                                        */
    uint16_t sample_period;        /* ms                                        */
    uint8_t  gain;                 /* 0–255                                     */
    uint8_t  trigger_button;       /* 0 = no trigger                            */
} FFB_SetEffect_t;                 /* 10 bytes                                  */

/* Report 0x07 — Set Condition (13 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  effect_block_index;
    uint8_t  param_block_offset;   /* axis index: 0=X                           */
    int16_t  cp_offset;            /* -10000..+10000                            */
    int16_t  positive_coefficient; /* 0..10000                                  */
    int16_t  negative_coefficient; /* 0..10000                                  */
    uint16_t positive_saturation;  /* 0..10000                                  */
    uint16_t dead_band;            /* 0..10000  (half-width, centred on cp_offset) */
} FFB_SetCondition_t;              /* 12 bytes                                  */

/* Report 0x08 — Set Constant Force (3 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t effect_block_index;
    int16_t magnitude;             /* -10000..+10000                            */
} FFB_SetConstantForce_t;          /* 3 bytes                                   */

/* Report 0x0B — Effect Operation (4 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  effect_block_index;
    uint8_t  operation;            /* FFB_OP_xxx                                */
    uint16_t loop_count;           /* 0xFFFF = infinite                         */
} FFB_EffectOperation_t;           /* 4 bytes                                   */

/* Report 0x0C — Device Control (1 byte) */
typedef struct __attribute__((packed)) {
    uint8_t control;               /* FFB_DC_xxx                                */
} FFB_DeviceControl_t;

/* Report 0x0D — Device Gain (1 byte) */
typedef struct __attribute__((packed)) {
    uint8_t device_gain;           /* 0–255                                     */
} FFB_DeviceGain_t;

/* ── Per-cycle torque contribution debug record ─────────────────────────────
 * Updated by ffb_compute_torque() every control cycle.
 * Read from any task — fields are float, writes are atomic on Cortex-M4.
 * Units: Nm throughout.                                                      */
typedef struct {
    /* DirectInput game effects (after game gain, before wheel gain)          */
    float di_constant;       /* Constant Force effect sum                     */
    float di_spring;         /* Spring (condition) effect sum                 */
    float di_damper;         /* Damper (condition) effect sum                 */
    float di_total;          /* DI sum after input filter × game gain         */

    /* Firmware physical effects (before wheel gain)                          */
    float phys_inertia;      /* Inertia  (−J × dω/dt)                        */
    float phys_friction;     /* Friction (Stribeck model)                     */
    float phys_damping;      /* Damping  (velocity-proportional)              */
    float phys_spring;       /* Always-on center spring                       */
    float phys_boost;        /* Center boost (fades with position)            */

    /* Post-processing scalars                                                 */
    float thermal_scale;     /* Current thermal derate factor [0..1]          */
    float game_gain;         /* device_gain / 255.0 (set by game via HID)     */
    float wheel_gain;        /* g_config.ffb_gain (set by user in GUI)        */

    /* Final                                                                   */
    float pre_lut;           /* Torque after slew/filter, before LUT          */
    float output;            /* Final output (before endstops in caller)      */
} FFB_DebugTorque_t;

/* Updated every call to ffb_compute_torque(). Zero-cost to ignore. */
extern FFB_DebugTorque_t g_ffb_debug;

/* ── Public API ──────────────────────────────────────────────────────────── */

/* Call once at startup */
void ffb_init(void);

/* Called from USBD_HID_EP0_RxReady when a PID report arrives.
 * report_id is the first byte; data points to the bytes after it. */
void ffb_process_report(uint8_t report_id, const uint8_t *data, uint16_t len);

/* Called from hid_app_integration every control cycle (~10 ms or faster).
 * pos_turns   = encoder position in turns (relative to home, + = right).
 * vel_turns_s = encoder velocity in turns/s.
 * Returns total torque command in Nm (clamped to ±ffb_max_torque). */
float ffb_compute_torque(float pos_turns, float vel_turns_s);

/* Fill PID Block Load response (GET_REPORT 0x0F).
 * Returns the number of bytes written to buf (including report_id). */
uint8_t ffb_get_block_load_report(uint8_t *buf, uint8_t buf_size);

/* Fill PID Pool response (GET_REPORT 0x10).
 * Returns the number of bytes written to buf. */
uint8_t ffb_get_pool_report(uint8_t *buf, uint8_t buf_size);

/* True when actuators are enabled (device control) */
bool ffb_actuators_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* FFB_PID_H */
