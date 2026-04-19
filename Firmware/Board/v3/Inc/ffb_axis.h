#include <stdint.h>

typedef struct FFB_Axis {
  int32_t position;
  int32_t velocity;
  int16_t acceleration;
  uint32_t prevUpdateTime;
} FFB_Axis;

int16_t FFB_Axis_Update(int32_t new_value);
