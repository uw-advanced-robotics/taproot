#ifndef __USER_MATH_UTILS_HPP__
#define __USER_MATH_UTILS_HPP__

#include <math.h>

namespace aruwlib
{

namespace algorithms
{

constexpr float PI = 3.1415926535897932384626f;

inline float degreesToRadians(float radians)
{
    return radians * PI / 180.0f;
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

}  // namespace algorithms

}  // namespace aruwlib

#endif
