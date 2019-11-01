#ifndef __USER_MATH_UTILS_HPP__
#define __USER_MATH_UTILS_HPP__

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

#endif
