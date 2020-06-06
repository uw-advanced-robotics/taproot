#include "math_user_utils.hpp"

#include <cstdint>

float aruwlib::algorithms::fastInvSqrt(float x)
{
    static_assert(sizeof(float) == 4, "fast inverse sqrt requires 32-bit float");
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = reinterpretCopy<float, int32_t>(y);
    i = 0x5f3759df - (i >> 1);
    y = reinterpretCopy<int32_t, float>(i);
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void aruwlib::algorithms::rotateVector(float* x, float* y, float radians)
{
    float x_temp = *x;
    *x = (*x) * cosf(radians) - *y * sinf(radians);
    *y = x_temp * sinf(radians) + *y * cosf(radians);
}
