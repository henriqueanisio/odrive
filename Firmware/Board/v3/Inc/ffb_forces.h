#include "ffb.h"
#include <stdint.h>

int32_t FFB_ConstantForce(const EffectCalcData *data);
int32_t FFB_RampForce(const EffectCalcData *data);
int32_t FFB_PeriodicForce(const EffectCalcData *data);
int32_t FFB_SpringForce(const EffectCalcData *data);
int32_t FFB_DamperForce(const EffectCalcData *data);
int32_t FFB_InertiaForce(const EffectCalcData *data);
int32_t FFB_FrictionForce(const EffectCalcData *data);

int32_t FFBEngine_LimitForce(void);

