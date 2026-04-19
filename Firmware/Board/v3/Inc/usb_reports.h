#include <stdint.h>

#ifndef __INCLUDE_USB_REPORT__
#define __INCLUDE_USB_REPORT__

#define HID_REPORT typedef struct __attribute__((packed, aligned(1)))

// Input reports
#define JOYSTICK_INPUT_REPORT_ID 1
HID_REPORT JoystickInputReport {
  uint8_t id;
  int16_t steering;
  uint16_t accelerator;
  uint16_t brake;
  uint32_t buttons;
}
JoystickInputReport;

void JoystickInputReport_Init(JoystickInputReport *report);

#define PID_STATE_REPORT_ID 2
HID_REPORT PIDStateReport {

  uint8_t id;
  struct {
    uint8_t devicePaused : 1;
    uint8_t actuatorsEnabled : 1;
    uint8_t actuatorPower : 1;
    uint8_t actuatorOverrideSwitch : 1;
  };

  struct {
    uint8_t effectPlaying : 1;
    uint8_t effectBlockIndex : 7;
  };
}
PIDStateReport;

void PIDStateReport_Init(PIDStateReport *report);

// Output Reports
// ...
#define SET_EFFECT_REPORT_ID 1

HID_REPORT PID_SetEffectReport {
  uint8_t id;
  uint8_t effectBlockIndex;
  uint8_t effectType;
  uint16_t duration;     // milliseconds
  uint16_t triggerRepeatInterval;
  uint16_t samplePeriod; // microseconds
  uint8_t gain;
  uint8_t enableAxis : 1;
}
PID_SetEffectReport;

#define SET_ENVELOPE_REPORT_ID 2

HID_REPORT PID_SetEnvelopeReport {
  uint8_t id;
  uint8_t effectBlockIndex;
  int16_t attackLevel;
  int16_t fadeLevel;
  uint16_t attackTime; // milliseconds
  uint16_t fadeTime;   // milliseconds
}
PID_SetEnvelopeReport;

#define SET_CONDITION_REPORT_ID 3

HID_REPORT PID_SetConditionReport {
  uint8_t id;
  uint8_t effectBlockIndex;
  int16_t cpOffset;
  int16_t positiveCoefficient;
  int16_t negativeCoefficient;
  uint16_t positiveSaturation;
  uint16_t negativeSaturation;
  uint16_t deadBand;
}
PID_SetConditionReport;

#define SET_PERIODIC_REPORT_ID 4

HID_REPORT PID_SetPeriodicReport {
  uint8_t id;
  uint8_t effectBlockIndex;
  uint16_t magnitude;
  int16_t offset;
  uint16_t phase;
  uint16_t period; // millisecond
}
PID_SetPeriodicReport;

#define SET_CONSTANT_FORCE_REPORT_ID 5

HID_REPORT PID_SetConstantForceReport {
  uint8_t id;
  uint8_t effectBlockIndex;
  int16_t magnitude;
}
PID_SetConstantForceReport;

#define SET_RAMP_FORCE_REPORT_ID 6

HID_REPORT PID_SetRampForceReport {
  uint8_t id;
  uint8_t effectBlockIndex;
  int16_t rampStart;
  int16_t rampEnd;
}
PID_SetRampForceReport;

#define EFFECT_OPERATION_REPORT_ID 10

enum EFFECT_OPERATION {
  EF_OP_EFFECT_START = 1,
  EF_OP_EFFECT_START_SOLO = 2,
  EF_OP_EFFECT_STOP = 3,
};

HID_REPORT PID_EffectOperationReport {
  uint8_t id;
  uint8_t effectBlockIndex;
  uint8_t effectOperation;
  uint8_t loopCount;
}
PID_EffectOperationReport;

#define DEVICE_BLOCK_FREE_REPORT_ID 11

HID_REPORT PID_BlockFreeReport {
  uint8_t id;
  uint8_t effectBlockIndex;
}
PID_BlockFreeReport;

#define DEVICE_CONTROL_REPORT_ID 12

enum PID_DeviceControl {
  DC_ENABLE_ACTUATORS = 1,
  DC_DISABLE_ACTUATORS = 2,
  DC_STOP_ALL_EFFECTS = 3,
  DC_DEVICE_RESET = 4,
  DC_DEVICE_PAUSE = 5,
  DC_DEVICE_CONTINUE = 6,
};

HID_REPORT PID_DeviceControlReport {
  uint8_t id;
  uint8_t control;
}
PID_DeviceControlReport;

#define DEVICE_GAIN_REPORT_ID 13

HID_REPORT PID_DeviceGainReport {
  uint8_t id;
  uint8_t gain;
}
PID_DeviceGainReport;

// Feature Reports
#define CREATE_NEW_EFFECT_REPORT_ID 7

typedef enum PID_EffectType {
  ET_CONSTANT_FORCE = 1,
  ET_RAMP,
  ET_SQUARE,
  ET_SINE,
  ET_TRIANGLE,
  ET_SAWTOOTH_UP,
  ET_SAWTOOTH_DOWN,
  ET_SPRING,
  ET_DAMPER,
  ET_INERTIA,
  ET_FRICTION,
} PID_EffectType;

HID_REPORT PID_CreateNewEffectReport {
  uint8_t id;
  uint8_t effectType;
  uint8_t byteCount;
}
PID_CreateNewEffectReport;

#define PID_BLOCK_LOAD_REPORT_ID 8

typedef enum PID_BlockLoadStatus {
  BLOCK_LOAD_SUCCESS = 1,
  BLOCK_LOAD_FULL = 2,
  BLOCK_LOAD_ERROR = 3,
} PID_BlockLoadStatus;

HID_REPORT PID_BlockLoadReport {
  uint8_t id;
  uint8_t effectBlockIndex;
  uint8_t blockLoadStatus;
  uint16_t ramPoolAvailable;
}
PID_BlockLoadReport;

#define PID_POOL_FEATURE_REPORT_ID 9
#define PID_DEVICE_MANAGED_POOL 1
#define PID_SHARED_PARAMETER_BLOCKS 1

HID_REPORT PID_PoolFeatureReport {
  uint8_t id;
  uint16_t ramPoolSize;
  uint8_t maxEffects;
  uint8_t deviceManagedPool : 1;
  uint8_t sharedParameterBlocks : 1;
}
PID_PoolFeatureReport;

void PIDPoolFeatureReport_Init(PID_PoolFeatureReport *report);
#endif
