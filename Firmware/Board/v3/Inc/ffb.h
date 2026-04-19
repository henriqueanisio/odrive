#ifndef INCLUDE_FFB_H
#define INCLUDE_FFB_H

#include <stdint.h>
#include "usb_reports.h"


typedef struct EnvelopeData {
  int16_t attackLevel;
  int16_t fadeLevel;
  uint16_t fadeTime;
  uint16_t attackTime;
} EnvelopeData;

typedef struct PeriodicForceData {
  uint16_t period;
} PeriodicForceData;

typedef struct ConditionalForceData {
  int16_t positiveCoefficient;
  int16_t negativeCoefficient;
  uint16_t positiveSaturation;
  uint16_t negativeSaturation;
  int16_t cpOffset;
  uint16_t deadBand;
} ConditionalForceData;

typedef struct EffectState {
  struct {
    uint8_t isAllocated : 1;
    uint8_t isPlaying : 1;
    uint8_t enableAxis : 1;
  };

  uint8_t effectType;
  uint8_t gain;

  uint32_t startTime;
  uint16_t duration;

  EnvelopeData envelopeData;
  struct {
    int16_t magnitude;
    int16_t offset;
    union {
      PeriodicForceData periodic;
      ConditionalForceData conditional;
    };
  } forceData;
} EffectState;

typedef struct EffectCalcData {
  uint32_t elapsed;
  int32_t periodTime;
  float fadeTimeC;
  float attackTimeC;
  float periodC;

  const EffectState effect;
} EffectCalcData;

void FFB_OnCreateNewEffect(const PID_CreateNewEffectReport* data);
volatile const PID_BlockLoadReport *FFB_GetPidBlockLoad(void);

void FFB_OnUsbData(uint8_t *buf, uint8_t len);

#endif
