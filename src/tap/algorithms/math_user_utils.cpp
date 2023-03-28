/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "math_user_utils.hpp"

#include <array>
#include <cassert>
#include <cstdint>

float tap::algorithms::fastInvSqrt(float x)
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

void tap::algorithms::rotateVector(float* x, float* y, float radians)
{
    float x_temp = *x;
    *x = (*x) * cosf(radians) - *y * sinf(radians);
    *y = x_temp * sinf(radians) + *y * cosf(radians);
}

float interpolateLinear2D(
    float** values,
    float* xmin,
    float* xmax,
    float* dx,
    float* ymin,
    float* ymax,
    float* dy,
    float xDes,
    float yDes)
{
    // check that xDes and yDes are in-bounds. No extrapolation
    assert(*dx != 0);
    assert(*dy != 0);
    if (*dx > 0)
    {
        assert(xDes < *xmin || xDes > *xmax);
    }
    else
    {
        assert(xDes > *xmin || xDes < *xmax);
    }
    if (*dy > 0)
    {
        assert(yDes < *ymin || yDes > *ymax);
    }
    else
    {
        assert(yDes > *ymin || yDes < *ymax);
    }

    int num_x = (*xmax - *xmin) / (*dx);
    float xScalingRatio =
        1 /
        *dx;  // we multiple the x range by xscalingratio to turn all x values into integers(ish)
    float xDesNormalized =
        (xDes - *xmin) *
        xScalingRatio;  // finds the value of x if the x-range were normalized to integers
    int xIndex = (int)floor(xDesNormalized);  // finds x1's index
    if (xIndex >= num_x) xIndex = num_x - 1;  // prevent OOBness
    if (xIndex < 0) xIndex = 0;
    float x1 = *xmin + xIndex * *dx;  // gets value from index
    float x2 = *xmin + (xIndex + 1) * *dx;

    int num_y = (*ymax - *ymin) / (*dy);
    float yScalingRatio = 1 / *dy;
    float yDesNormalized = (yDes - *ymin) * yScalingRatio;
    int yIndex = (int)floor(yDesNormalized);
    if (yIndex >= num_y) yIndex = num_y - 1;
    if (yIndex < 0) yIndex = 0;
    float y1 = *ymin + yIndex * *dy;
    float y2 = *ymin + (yIndex + 1) * *dy;

    float q11, q12, q21, q22;  // x1y1, x1y2, x2y1, x2y2
    q11 = *(*(values + xIndex) + yIndex);
    q12 = *(*(values + xIndex) + yIndex + 1);
    q21 = *(*(values + xIndex + 1) + yIndex);
    q22 = *(*(values + xIndex + 1) + yIndex + 1);

    float x2x, y2y, yy1, xx1;
    x2x = x2 - xDes;
    y2y = y2 - yDes;
    yy1 = yDes - y1;
    xx1 = xDes - x1;
    return 1.0 / (*dx * *dy) *
           (q11 * x2x * y2y + q21 * xx1 * y2y + q12 * x2x * yy1 + q22 * xx1 * yy1);
}
