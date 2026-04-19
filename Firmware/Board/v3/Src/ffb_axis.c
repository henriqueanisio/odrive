#include "ffb_axis.h"
#include "util.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MOV_AVG_LEVEL 6
#define MOV_AVG_SIZE (1 << MOV_AVG_LEVEL)

typedef struct MovingAverage32 {
  int32_t values[MOV_AVG_SIZE];
  int32_t total;
  uint8_t index;
} MovingAverage32;

int32_t MOV_AVG_SetValue(MovingAverage32 *avg, int32_t value) {
  avg->total -= avg->values[avg->index];
  avg->total += value;
  avg->values[avg->index] = value;

  avg->index = (avg->index + 1) % MOV_AVG_SIZE;

  return (avg->total >> MOV_AVG_LEVEL);
}

int32_t MOV_AVG_GetValue(MovingAverage32 *avg) {
  return avg->values[avg->index];
}

void MOV_AVG_Reset(MovingAverage32 *avg) {
  avg->index = 0;
  avg->total = 0;
}

volatile FFB_Axis FFB_axis;

MovingAverage32 position_filter;
MovingAverage32 velocity_filter;
MovingAverage32 acceleration_filter;

int16_t FFB_Axis_Update(int32_t value) {
  int32_t lastPosition = FFB_axis.position;
  value = MOV_AVG_SetValue(&position_filter, value);
  FFB_axis.position = value;

  uint32_t tmpUs = HAL_GetTick();
  int32_t td = tmpUs - FFB_axis.prevUpdateTime;
  FFB_axis.prevUpdateTime = tmpUs;

  int32_t newVelocity;
  newVelocity =
      MOV_AVG_SetValue(&velocity_filter, ((value - lastPosition) * (2 << 15)) / td);

  FFB_axis.acceleration = MOV_AVG_SetValue(
      &acceleration_filter, ((newVelocity - FFB_axis.velocity) * (2 << 15)) / td);
  FFB_axis.velocity = newVelocity;

  return (int16_t)constrain(value, -32768, 32767);
}
