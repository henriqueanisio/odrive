#include "trig.h"

#define FIXED(x, n) ((uint16_t)((float)(x) * ((uint32_t)1 << (n)) + .5))

static uint16_t mul_fix_u16(uint16_t x, uint16_t y)
{
    return ((uint32_t)x * y + 0x8000) >> 16;
}

static uint16_t mul_high_bytes(uint16_t x, uint16_t y)
{
    return (uint8_t)(x >> 8) * (uint8_t)(y >> 8);
}

int16_t cos_fix(uint16_t x)
{
    uint16_t y, s;
    uint8_t i = (x >> 8) & 0xc0;
    x = (x & 0x3fff) << 1;
    if (i & 0x40) x = FIXED(1, 15) - x;
    x = mul_fix_u16(x, x) << 1;
    y = FIXED(1, 15) - x;
    s = FIXED(0.23361, 16) - mul_high_bytes(FIXED(0.019531, 17), x);
    s = FIXED(1, 15) - mul_fix_u16(x, s);
    s = mul_fix_u16(y, s);
    return (i == 0x40 || i == 0x80) ? -(int16_t)s : (int16_t)s;
}

int16_t sin_fix(uint16_t x)
{
    return cos_fix(0xc000 + x);
}

static uint16_t atan_fix(uint16_t x)
{
    uint16_t s;
    s = FIXED(0.0625, 18);
    s = FIXED(0.270519, 17) - mul_high_bytes(x, s);
    s = FIXED(0.299045, 16) - mul_fix_u16(x, s);
    s = FIXED(0.271553, 15) + mul_fix_u16(x, s);
    s = FIXED(1, 14) + mul_fix_u16(FIXED(1, 15) - x, s);
    return mul_fix_u16(x, s);
}

int16_t atan2_fix(int16_t y, int16_t x)
{
    static const uint8_t axis[8] = {0x00, 0x40, 0x00, 0xc0,
                                    0x80, 0x40, 0x80, 0xc0};
    uint8_t octant = 0;
    if (x < 0) { x = -x; octant |= 4; }
    if (y < 0) { y = -y; octant |= 2; }
    if (y > x) { int16_t tmp = x; x = y; y = tmp; octant |= 1; }
    uint16_t angle = atan_fix((((uint32_t)y) << 15) / (uint32_t)x);
    if ((octant ^ (octant >> 1) ^ (octant >> 2)) & 1) angle = (uint16_t)(-(int16_t)angle);
    return (int16_t)(angle + ((uint16_t)axis[octant] << 8));
}
