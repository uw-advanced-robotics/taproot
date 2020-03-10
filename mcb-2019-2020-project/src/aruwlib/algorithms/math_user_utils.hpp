#ifndef __USER_MATH_UTILS_HPP__
#define __USER_MATH_UTILS_HPP__

#include <cmath>
#include <cstring>

namespace aruwlib
{

namespace algorithms
{

constexpr float PI = 3.1415926535897932384626f;

inline float degreesToRadians(float degrees)
{
    return degrees * PI / 180.0f;
}

inline float radiansToDegrees(float radians)
{
    return radians * 180.f / PI;
}

inline bool compareFloatClose(float val1, float val2, float epsilon)
{
    return fabsf(val1 - val2) < epsilon;
}

template< typename T >
T limitVal(T val, T min, T max)
{
    if (min >= max)
    {
        return val;
    }
    if (val < min)
    {
        return min;
    }
    else if (val > max)
    {
        return max;
    }
    else
    {
        return val;
    }
}

inline float lowPassFilter(float prevValue, float newValue, float alpha)
{
    if (alpha < 0.0f || alpha > 1.0f) {
        return newValue;
    }
    return alpha * newValue + (1.0f - alpha) * prevValue;
}

template <typename From, typename To>
To reinterpretCopy(From from) {
    static_assert(sizeof(From) == sizeof(To), "can only reinterpret-copy types of the same size");
    To result;
    memcpy(static_cast<void*>(&result), static_cast<void*>(&from), sizeof(To));
    return result;
}

/**
  * @brief     Fast inverse square-root, to calculate 1/Sqrt(x)
  * @param[in] input:x
  * @retval    1/Sqrt(x)
  */
float fastInvSqrt(float x);

}  // namespace algorithms

}  // namespace aruwlib

#endif
