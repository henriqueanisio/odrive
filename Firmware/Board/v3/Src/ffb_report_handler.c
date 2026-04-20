#include "config.h"

#include "ffb.h"
#include "ffb_engine.h"

#include <stdint.h>
#include <stdlib.h>

#define SIZE_EFFECT sizeof(EffectState)

extern volatile PID_BlockLoadReport pidBlockLoad;
extern volatile PIDStateReport g_state;
extern volatile uint8_t g_deviceGain;

extern void hid_notify_pid_state_change(bool enabled);

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
    hid_notify_pid_state_change(true);
    return;
  case DC_DISABLE_ACTUATORS:
    g_state.actuatorsEnabled = FALSE;
    hid_notify_pid_state_change(true);
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

void FFB_OnUsbData(uint8_t *buf, uint16_t len)
{
  if (len < 2)
    return;

  uint8_t report_id = buf[0];
  const uint8_t *data = &buf[1];
  uint16_t payload_len = len - 1;

  switch (report_id)
  {
  case SET_EFFECT_REPORT_ID:
    if (payload_len < sizeof(PID_SetEffectReport)) return;
    {
      PID_SetEffectReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_SetEffect(&tmp);
    }
    return;

  case SET_ENVELOPE_REPORT_ID:
    if (payload_len < sizeof(PID_SetEnvelopeReport)) return;
    {
      PID_SetEnvelopeReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_SetEnvelope(&tmp);
    }
    return;

  case SET_CONDITION_REPORT_ID:
    if (payload_len < sizeof(PID_SetConditionReport)) return;
    {
      PID_SetConditionReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_SetConditionReport(&tmp);
    }
    return;

  case SET_PERIODIC_REPORT_ID:
    if (payload_len < sizeof(PID_SetPeriodicReport)) return;
    {
      PID_SetPeriodicReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_SetPeriodic(&tmp);
    }
    return;

  case SET_CONSTANT_FORCE_REPORT_ID:
    if (payload_len < sizeof(PID_SetConstantForceReport)) return;
    {
      PID_SetConstantForceReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_SetConstantForce(&tmp);
    }
    return;

  case SET_RAMP_FORCE_REPORT_ID:
    if (payload_len < sizeof(PID_SetRampForceReport)) return;
    {
      PID_SetRampForceReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_SetRampForce(&tmp);
    }
    return;

  case CREATE_NEW_EFFECT_REPORT_ID:
    if (payload_len < sizeof(PID_CreateNewEffectReport)) return;
    {
      PID_CreateNewEffectReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      FFB_OnCreateNewEffect(&tmp);
    }
    return;

  case EFFECT_OPERATION_REPORT_ID:
    if (payload_len < sizeof(PID_EffectOperationReport)) return;
    {
      PID_EffectOperationReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_EffectOperation(&tmp);
    }
    return;

  case DEVICE_BLOCK_FREE_REPORT_ID:
    if (payload_len < sizeof(PID_BlockFreeReport)) return;
    {
      PID_BlockFreeReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_BlockFree(&tmp);
    }
    return;

  case DEVICE_CONTROL_REPORT_ID:
    if (payload_len < sizeof(PID_DeviceControlReport)) return;
    {
      PID_DeviceControlReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_DeviceControl(&tmp);
    }
    return;

  case DEVICE_GAIN_REPORT_ID:
    if (payload_len < sizeof(PID_DeviceGainReport)) return;
    {
      PID_DeviceGainReport tmp;
      memcpy(&tmp, data, sizeof(tmp));
      On_DeviceGain(&tmp);
    }
    return;

  default:
    return;
  }
}
