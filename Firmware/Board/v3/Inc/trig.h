#ifndef TRIG_H
#define TRIG_H

#include <stdint.h>

/* Fixed-point trigonometry (argument in units of 2*PI/2^16,
 * result in units of 1/2^14, range [-2^14 : 2^14]). */
int16_t cos_fix(uint16_t x);
int16_t sin_fix(uint16_t x);
int16_t atan2_fix(int16_t y, int16_t x);

#endif /* TRIG_H */
