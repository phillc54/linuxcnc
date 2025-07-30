#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define CLAMP(x, low, high) (MIN(MAX(x, low), high))
#define LIMIT(x, lowhigh) (CLAMP((x), (-lowhigh), (lowhigh)))
#define SIGN(a) (((a) < 0.0) ? (-1.0) : (((a) > 0.0) ? (1.0) : (0.0)))
#define SIGN2(a, b) (CLAMP((a) / (b), -1.0, 1.0))

#define hypot2(x, y) (sqrt((x)*(x) + (y)*(y)))
#define hypot3(x, y, z) (sqrt((x)*(x) + (y)*(y) + (z)*(z)))
#define hypot4(w, x, y, z) (sqrt((w)*(w) + (x)*(x) + (y)*(y) + (z)*(z)))

#define INT32_MOD ((int64_t)(0xFFFFFFFF))
#define S32_OVERFLOW_THRESHOLD ((int64_t)(1 << 30))

static inline int64_t unwrap_s32_delta(int32_t value_from_s32, int32_t prev_value_from_s32, int homing_active)
{
    int64_t delta = (int64_t)value_from_s32 - (int64_t)prev_value_from_s32;
    if (homing_active) {
        return delta;
    } else if (delta > S32_OVERFLOW_THRESHOLD) {
        // Large positive delta means rollover in the negative direction
        return delta - INT32_MOD;
    } else if (delta < -S32_OVERFLOW_THRESHOLD) {
        // Large negative delta means rollover in positive direction
        return delta + INT32_MOD;
    }
    return delta;
}

static inline int64_t find_overflow_correction_s32(int32_t value_from_s32, int32_t prev_value_from_s32)
{
    int64_t delta = (int64_t)value_from_s32 - (int64_t)prev_value_from_s32;
    if (delta > S32_OVERFLOW_THRESHOLD) {
        // Large positive delta means rollover in the negative direction
        return -INT32_MOD;
    } else if (delta < -S32_OVERFLOW_THRESHOLD) {
        // Large negative delta means rollover in positive direction
        return INT32_MOD;
    }
    return 0;
}

static inline int32_t wrap_s32(double value) {return (int32_t)(int64_t)value;}

#ifdef __cplusplus
}
#endif

#endif // MATH_UTIL_H
