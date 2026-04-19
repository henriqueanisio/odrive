#include "config.h"

#include "ffb_axis.h"
#include "ffb_forces.h"

#include "trig.h"
#include "util.h"

#include <stdint.h>
#include <stdlib.h>

int32_t envelope(const EffectCalcData *data) {

  EnvelopeData envelope = data->effect.envelopeData;
  int16_t magnitude = data->effect.forceData.magnitude;
  uint16_t duration = data->effect.duration;
  uint32_t time = data->elapsed;

  if (time < envelope.attackTime) {
    return sign(magnitude) *
           (int32_t)(envelope.attackLevel +
                     (float)(abs(magnitude) - envelope.attackLevel) *
                         (float)time * data->attackTimeC);
  }

  if (duration == DURATION_INF)
    return magnitude;

  if (envelope.fadeTime && time > (int32_t)duration - envelope.fadeTime) {
    return sign(magnitude) *
           (int32_t)(envelope.fadeLevel +
                     (float)(abs(magnitude) - envelope.fadeLevel) *
                         (float)(duration - time) * data->fadeTimeC);
  }

  return magnitude;
}

int32_t square(const EffectCalcData *data, int32_t magnitude) {
  int32_t half_period = data->effect.forceData.periodic.period >> 1;
  if (data->periodTime < half_period)
    return magnitude;
  else
    return -magnitude;
}

int32_t sine_fix(const EffectCalcData *data, int32_t magnitude) {
  return magnitude * sin_fix(data->periodTime * data->periodC) >> 14;
}

int32_t triangle(const EffectCalcData *data, int32_t magnitude) {
  int32_t half_period = data->effect.forceData.periodic.period >> 1;
  if (data->periodTime < half_period)
    return -magnitude + (int32_t)(magnitude * data->periodTime * data->periodC);
  else
    return 3 * magnitude -
           (int32_t)(magnitude * data->periodTime * data->periodC);
}

int32_t st_down(const EffectCalcData *data, int32_t magnitude) {
  return magnitude - (int32_t)(magnitude * data->periodTime * data->periodC);
}

int32_t FFB_ConstantForce(const EffectCalcData *data) { return envelope(data); }

int32_t FFB_RampForce(const EffectCalcData *data) {
  int32_t magnitude = envelope(data);
  int32_t offset = data->effect.forceData.offset;

  if (data->effect.duration == DURATION_INF) {
    return offset - magnitude;
  }

  return offset +
         (int32_t)((float)magnitude * (float)data->elapsed * data->periodC) -
         magnitude;
}

int32_t FFB_PeriodicForce(const EffectCalcData *data) {
  int32_t offset = data->effect.forceData.offset;

  int32_t magnitude = envelope(data);

  switch (data->effect.effectType) {
  case ET_SQUARE:
    return offset + square(data, magnitude);
  case ET_SINE:
    return offset + sine_fix(data, magnitude);
  case ET_TRIANGLE:
    return offset + triangle(data, magnitude);
  case ET_SAWTOOTH_DOWN:
    return offset + st_down(data, magnitude);
  case ET_SAWTOOTH_UP:
    return offset - st_down(data, magnitude);
  default:
    return 0;
  }
}

extern volatile FFB_Axis FFB_axis;

int16_t GetAxisPosition(void) { return FFB_axis.position; }

int16_t GetAxisVelocity(void) { return FFB_axis.velocity; }

int16_t GetAxisAcceleration(void) { return FFB_axis.acceleration; }

int32_t FFB_SpringForce(const EffectCalcData *data) {
  int32_t tempForce;

  const ConditionalForceData *cond = &data->effect.forceData.conditional;

  int32_t position = GetAxisPosition();

  uint16_t deadBand = FFB_DEAD_ZONE + cond->deadBand;

  if (position < (cond->cpOffset - deadBand)) {
    tempForce = ((position - ((int32_t)cond->cpOffset - deadBand)) *
                 cond->negativeCoefficient) >>
                8;
    tempForce = constrain(tempForce, -cond->negativeSaturation,
                          cond->negativeSaturation);
  } else if (position > (cond->cpOffset + deadBand)) {
    tempForce = ((position - ((int32_t)cond->cpOffset + deadBand)) *
                 cond->positiveCoefficient) >>
                8;
    tempForce = constrain(tempForce, -cond->positiveSaturation,
                          cond->positiveSaturation);
  } else
    return 0;

  return tempForce;
}

#define MAX_VELOCITY_DAMPER_C 1

int32_t FFB_DamperForce(const EffectCalcData *data) {
  int32_t tempForce;

  int32_t velocity = GetAxisVelocity() * MAX_VELOCITY_DAMPER_C;
  const ConditionalForceData *cond = &data->effect.forceData.conditional;

  if (velocity < (cond->cpOffset - (int16_t)cond->deadBand)) {
    tempForce = ((velocity - (cond->cpOffset - (int32_t)cond->deadBand)) *
                 cond->negativeCoefficient) >>
                14;
    tempForce = constrain(tempForce, -(int16_t)cond->negativeSaturation,
                          cond->negativeSaturation);
  } else if (velocity > (cond->cpOffset + (int16_t)cond->deadBand)) {
    tempForce = ((velocity - (cond->cpOffset + (int32_t)cond->deadBand)) *
                 cond->positiveCoefficient) >>
                14;
    tempForce = constrain(tempForce, -(int16_t)cond->positiveSaturation,
                          cond->positiveSaturation);
  } else
    return 0;

  return tempForce;
}

#define MAX_ACCELERATION_INERTIA_C 1

int32_t FFB_InertiaForce(const EffectCalcData *data) {
  int32_t tempForce;
  const ConditionalForceData *cond = &data->effect.forceData.conditional;

  // inertia effect works only for deceleration
  if (sign(FFB_axis.acceleration) == sign(FFB_axis.velocity))
    return 0;

  int16_t acceleration = GetAxisAcceleration() * MAX_ACCELERATION_INERTIA_C;

  if (acceleration < (cond->cpOffset - (int16_t)cond->deadBand)) {
    tempForce = ((acceleration - (cond->cpOffset - (int32_t)cond->deadBand)) *
                 cond->negativeCoefficient) >>
                14;

    // Saturation ignored if is 0
    if (cond->negativeSaturation)
      tempForce = constrain(tempForce, -(int16_t)cond->negativeSaturation,
                            cond->negativeSaturation);
  } else if (acceleration > (cond->cpOffset + (int16_t)cond->deadBand)) {
    tempForce = ((acceleration - (cond->cpOffset + (int32_t)cond->deadBand)) *
                 cond->positiveCoefficient) >>
                14;

    if (cond->positiveSaturation)
      tempForce = constrain(tempForce, -(int16_t)cond->positiveSaturation,
                            cond->positiveSaturation);
  } else
    return 0;

  return tempForce;
}

#define MAX_VELOCITY_FRICTION_C 1

int32_t FFB_FrictionForce(const EffectCalcData *data) {

  const ConditionalForceData *cond = &data->effect.forceData.conditional;

  int32_t velocity = GetAxisVelocity() * MAX_VELOCITY_FRICTION_C;
  if (velocity == 0)
    return 0;

  int32_t coefficient;
  int32_t tVelocity;

  if (velocity > 0) {
    coefficient = cond->positiveCoefficient;
    tVelocity = velocity - cond->deadBand;
  } else {
    coefficient = -cond->negativeCoefficient;
    tVelocity = -velocity - cond->deadBand;
  }

  if (tVelocity > 256) {
    return coefficient;
  } else if (tVelocity > 0) {
    return (tVelocity * coefficient) >> 8;
  }

  return 0;
}

#define INERTIA_POWER 600

int32_t FFBEngine_LimitForce(void) {
  int32_t out = constrain(33000 - FFB_axis.position, -1000000, 0);
  out += constrain(-FFB_axis.position - 33000, 0, 1000000);
  return out * 16;
}
