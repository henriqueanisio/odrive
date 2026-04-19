#include "ffb.h"
#include <stdint.h>

volatile EffectState* GetEffectById(uint8_t id);

uint8_t GetNextFreeEffect(void);
void AllocateEffect(uint8_t id);
void StartEffect(uint8_t id);
void StopEffect(uint8_t id);
void FreeEffect(uint8_t id);
void FreeAllEffects(void);
void StopAllEffects(void);
int16_t FFBEngine_CalculateForce(void);
int32_t FFBEngine_CalculateEffectForce(uint8_t id, uint32_t time);

