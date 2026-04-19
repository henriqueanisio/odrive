#include "config.h"

#include "ffb.h"
#include "ffb_engine.h"

#include <stdint.h>
#include <stdlib.h>

#define SIZE_EFFECT sizeof(EffectState)

extern volatile PID_BlockLoadReport pidBlockLoad;
extern volatile PIDStateReport g_state;
extern volatile uint8_t g_deviceGain;

volatile const PID_BlockLoadReport *FFB_GetPidBlockLoad(void) {
  return &pidBlockLoad;
}

void FFB_OnCreateNewEffect(const PID_CreateNewEffectReport* data) {
  uint8_t id = GetNextFreeEffect();
  pidBlockLoad.effectBlockIndex = id;

  if (id == 0) {
    pidBlockLoad.blockLoadStatus = BLOCK_LOAD_FULL;
  } else {
    pidBlockLoad.blockLoadStatus = BLOCK_LOAD_SUCCESS;
    pidBlockLoad.ramPoolAvailable -= SIZE_EFFECT;
    volatile EffectState* effect = GetEffectById(id);
    effect->effectType = data->effectType;
  }

}

void On_SetEffect(const PID_SetEffectReport *data) {
  volatile EffectState *effect = GetEffectById(data->effectBlockIndex);

  effect->effectType = data->effectType;
  effect->duration = data->duration;
  effect->gain = data->gain;
  effect->enableAxis = data->enableAxis;
}

void On_SetEnvelope(const PID_SetEnvelopeReport *data) {
  volatile EffectState *effect = GetEffectById(data->effectBlockIndex);

  effect->envelopeData.attackLevel = (int16_t)abs(data->attackLevel);
  effect->envelopeData.fadeLevel = (int16_t)abs(data->fadeLevel);
  effect->envelopeData.attackTime = data->attackTime;
  effect->envelopeData.fadeTime = data->fadeTime;
}

void On_SetConditionReport(const PID_SetConditionReport *data) {
  volatile EffectState *effect = GetEffectById(data->effectBlockIndex);

  effect->forceData.conditional.cpOffset = data->cpOffset;
  effect->forceData.conditional.positiveCoefficient = data->positiveCoefficient;
  effect->forceData.conditional.negativeCoefficient = data->negativeCoefficient;
  effect->forceData.conditional.positiveSaturation = data->positiveSaturation;
  effect->forceData.conditional.negativeSaturation = data->negativeSaturation;
  effect->forceData.conditional.deadBand = data->deadBand;
}

void On_SetPeriodic(const PID_SetPeriodicReport *data) {
  volatile EffectState *effect = GetEffectById(data->effectBlockIndex);

  effect->forceData.magnitude = data->magnitude;
  effect->forceData.offset = data->offset;
  effect->forceData.periodic.period = data->period;
}

void On_SetConstantForce(const PID_SetConstantForceReport *data) {
  volatile EffectState *effect = GetEffectById(data->effectBlockIndex);
  effect->forceData.magnitude = data->magnitude;
}

void On_SetRampForce(const PID_SetRampForceReport *data) {
  volatile EffectState *effect = GetEffectById(data->effectBlockIndex);
  int32_t tmp = data->rampEnd + data->rampStart;
  effect->forceData.offset = (uint16_t)(tmp >> 1);

  tmp = data->rampEnd - data->rampStart;
  effect->forceData.magnitude = (uint16_t)(tmp >> 1);
}

void On_EffectOperation(const PID_EffectOperationReport *data) {
  volatile EffectState *effect = GetEffectById(data->effectBlockIndex);

  switch (data->effectOperation) {
  case EF_OP_EFFECT_START:
    if (data->loopCount > 0)
      effect->duration *= data->loopCount;
    if (data->loopCount == 0xFF)
      effect->duration = DURATION_INF;
    StartEffect(data->effectBlockIndex);
    return;

  case EF_OP_EFFECT_START_SOLO:
    StopAllEffects();
    StartEffect(data->effectBlockIndex);
    return;

  case EF_OP_EFFECT_STOP:
    StopEffect(data->effectBlockIndex);
    return;
  default:
    return;
  }
}

void On_BlockFree(const PID_BlockFreeReport *data) {
  uint8_t eid = data->effectBlockIndex;

  if (eid == 0xFF) {
    FreeAllEffects();
  } else {
    FreeEffect(eid);
  }
}

void On_DeviceControl(const PID_DeviceControlReport *data) {
  switch (data->control) {
  case DC_ENABLE_ACTUATORS:
    g_state.actuatorsEnabled = TRUE;
    return;
  case DC_DISABLE_ACTUATORS:
    g_state.actuatorsEnabled = FALSE;
    return;
  case DC_STOP_ALL_EFFECTS:
    StopAllEffects();
    return;
  case DC_DEVICE_RESET:
    FreeAllEffects();
    g_state.devicePaused = 0;
    g_state.actuatorsEnabled = TRUE;
    return;
  case DC_DEVICE_PAUSE:
    g_state.devicePaused = 1;
    return;
  case DC_DEVICE_CONTINUE:
    g_state.devicePaused = 0;
    return;
  default:
    return;
  }
}

void On_DeviceGain(const PID_DeviceGainReport *data) {
  g_deviceGain = data->gain;
}

void FFB_OnUsbData(uint8_t *buf, uint8_t len) {
  if (len < 2)
    return;

  uint8_t report_id = buf[0];
  switch (report_id) {
  case SET_EFFECT_REPORT_ID:
    On_SetEffect((const PID_SetEffectReport *)buf);
    return;
  case SET_ENVELOPE_REPORT_ID:
    On_SetEnvelope((const PID_SetEnvelopeReport *)buf);
    return;
  case SET_CONDITION_REPORT_ID:
    On_SetConditionReport((const PID_SetConditionReport *)buf);
    return;
  case SET_PERIODIC_REPORT_ID:
    On_SetPeriodic((const PID_SetPeriodicReport *)buf);
    return;
  case SET_CONSTANT_FORCE_REPORT_ID:
    On_SetConstantForce((const PID_SetConstantForceReport *)buf);
    return;
  case SET_RAMP_FORCE_REPORT_ID:
    On_SetRampForce((const PID_SetRampForceReport *)buf);
    return;
  case CREATE_NEW_EFFECT_REPORT_ID:
    FFB_OnCreateNewEffect((const PID_CreateNewEffectReport*)buf);
    return;
  case EFFECT_OPERATION_REPORT_ID:
    On_EffectOperation((const PID_EffectOperationReport *)buf);
    return;
  case DEVICE_BLOCK_FREE_REPORT_ID:
    On_BlockFree((const PID_BlockFreeReport *)buf);
    return;
  case DEVICE_CONTROL_REPORT_ID:
    On_DeviceControl((const PID_DeviceControlReport *)buf);
    return;
  case DEVICE_GAIN_REPORT_ID:
    On_DeviceGain((const PID_DeviceGainReport *)buf);
    return;
  default:
    return;
  }
}
