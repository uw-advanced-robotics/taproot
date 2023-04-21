/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <gtest/gtest.h>

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

TEST(MathUserUtils, compareFloatClose_within_epsilon)
{
    EXPECT_TRUE(compareFloatClose(1, 2, 2));
    EXPECT_TRUE(compareFloatClose(1, 2, 1));
    EXPECT_TRUE(compareFloatClose(2, 1, 1));
}

TEST(MathUserUtils, compareFloatClose_outside_epsilon)
{
    EXPECT_FALSE(compareFloatClose(1, 2, 0.5f));
    EXPECT_FALSE(compareFloatClose(2, 1, 0.5f));
    EXPECT_FALSE(compareFloatClose(1, 2, 0.99f));
    EXPECT_FALSE(compareFloatClose(1, 3, 1));
    EXPECT_FALSE(compareFloatClose(3, 1, 1));
}

TEST(MathUserUtils, limitVal_val_gt_max) { EXPECT_EQ(30, limitVal<int>(45, -54, 30)); }

TEST(MathUserUtils, limitVal_val_lt_min) { EXPECT_EQ(-54, limitVal<int>(-65, -54, 30)); }

TEST(MathUserUtils, limitVal_val_within_min_max) { EXPECT_EQ(42, limitVal<int>(42, -54, 60)); }

TEST(MathUserUtils, limitVal_min_max_equal) { EXPECT_EQ(2, limitVal<int>(4, 2, 2)); }

TEST(MathUserUtils, limitVal_min_gt_max) { EXPECT_EQ(42, limitVal<int>(42, 60, 50)); }

TEST(MathUserUtils, lowPassFilter_alpha_1) { EXPECT_EQ(43, lowPassFilter(42, 43, 1)); }

TEST(MathUserUtils, lowPassFilter_alpha_0) { EXPECT_EQ(42, lowPassFilter(42, 43, 0)); }

TEST(MathUserUtils, lowPassFilter_alpha_not_valid)
{
    EXPECT_EQ(42, lowPassFilter(0, 42, -1));
    EXPECT_EQ(42, lowPassFilter(0, 42, 2));
}

TEST(MathUserUtils, lowPassFilter_valid_alpha)
{
    EXPECT_NEAR(42.5f, lowPassFilter(42, 43, 0.5f), 1E-5);
}

TEST(MathUserUtils, reinterpretCopy_int_to_unsigned)
{
    auto out = reinterpretCopy<uint8_t, int8_t>(0xff);
    EXPECT_EQ(-1, out);
}

TEST(MathUserUtils, fastInvSqrt_simple)
{
    for (size_t x = 1; x < 10; x++)
    {
        EXPECT_NEAR(1.0f / sqrtf(x), fastInvSqrt(x), 1E-1);
    }
}

TEST(MathUserUtils, rotateVector_simple)
{
    float x = 10, y = 0;
    rotateVector(&x, &y, M_PI);
    EXPECT_NEAR(-10, x, 1E-5);
    EXPECT_NEAR(0, y, 1E-5);

    x = 10, y = 0;
    rotateVector(&x, &y, M_PI_2);
    EXPECT_NEAR(0, x, 1E-5);
    EXPECT_NEAR(10, y, 1E-5);

    x = 0, y = 10;
    rotateVector(&x, &y, M_PI_2);
    EXPECT_NEAR(-10, x, 1E-5);
    EXPECT_NEAR(0, y, 1E-5);
}

TEST(MathUserUtils, ceil_simple) { static_assert(tap::algorithms::ceil(42.5) == 43); }

TEST(MathUserUtils, getSign_simple)
{
    EXPECT_EQ(0, getSign(0.0f));
    EXPECT_EQ(1, getSign(42.0f));
    EXPECT_EQ(-1, getSign(-42.0f));
}

TEST(MathUserUtils, bilinear_interpolate_simple)
{
    const std::array<std::array<float, 5>, 2> values = {{{1, 2, 3, 4, 5}, {2, 3, 4, 5, 6}}};
    EXPECT_NEAR(1.5, interpolateLinear2D(values, 5, 10, 5, .1, .5, .1, 7.5, 0.1), 1E-3);
    EXPECT_NEAR(2.0, interpolateLinear2D(values, 5, 10, 5, .1, .5, .1, 7.5, 0.15), 1E-3);
    EXPECT_NEAR(2.5, interpolateLinear2D(values, 5, 10, 5, .1, .5, .1, 10, 0.15), 1E-3);
}

TEST(MathUserUtils, bilinear_interpolate_exceptions)
{
    const std::array<std::array<float, 5>, 2> values = {{{1, 2, 3, 4, 5}, {2, 3, 4, 5, 6}}};
    // checks for extrapolation
    EXPECT_NEAR(2.0, interpolateLinear2D(values, 5, 10, 5, .1, .5, .1, 500, -50), 1E-3);
    EXPECT_NEAR(3.0, interpolateLinear2D(values, 5, 10, 5, .1, .5, .1, 5, .3), 1E-6);
}