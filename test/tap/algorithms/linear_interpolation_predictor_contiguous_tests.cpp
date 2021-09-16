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

#include <gtest/gtest.h>

#include "tap/algorithms/linear_interpolation_predictor_contiguous.hpp"

using namespace tap::algorithms;

TEST(
    LinearInterpolationPredictorContiguous,
    update_return_same_value_as_reset_if_time_before_reset_time_used)
{
    LinearInterpolationPredictorContiguous li(0, 10);

    li.reset(5, 10);
    li.update(6, 5);
    EXPECT_FLOAT_EQ(5, li.getInterpolatedValue(100));
    li.update(3, 1);
    EXPECT_FLOAT_EQ(5, li.getInterpolatedValue(100));
}

TEST(LinearInterpolationPredictorContiguous, after_reset_return_reset_initial_val)
{
    LinearInterpolationPredictorContiguous li(0, 10);

    li.reset(3, 5);
    EXPECT_FLOAT_EQ(3, li.getInterpolatedValue(100));
    li.reset(7, 1);
    EXPECT_FLOAT_EQ(7, li.getInterpolatedValue(100));
}

TEST(LinearInterpolationPredictorContiguous, normal_operation_no_wrapping)
{
    LinearInterpolationPredictorContiguous li(0, 10);

    li.reset(3, 10);
    EXPECT_FLOAT_EQ(3, li.getInterpolatedValue(15));
    li.update(4, 20);
    EXPECT_FLOAT_EQ(4.5, li.getInterpolatedValue(25));
    li.update(6, 25);
    EXPECT_FLOAT_EQ(6.4, li.getInterpolatedValue(26));
    li.update(4, 26);
    EXPECT_FLOAT_EQ(2, li.getInterpolatedValue(27));
}

TEST(LinearInterpolationPredictorContiguous, normal_operation_wrapping)
{
    LinearInterpolationPredictorContiguous li(0, 10);

    li.reset(1, 10);
    EXPECT_FLOAT_EQ(1, li.getInterpolatedValue(11));
    li.update(9, 12);
    EXPECT_FLOAT_EQ(7, li.getInterpolatedValue(14));

    li.reset(1, 10);
    li.update(0.5, 11);
    EXPECT_FLOAT_EQ(9.5, li.getInterpolatedValue(13));

    li.reset(9, 10);
    li.update(1, 11);
    EXPECT_FLOAT_EQ(3, li.getInterpolatedValue(12));

    li.reset(9, 10);
    li.update(9.5, 11);
    EXPECT_FLOAT_EQ(0.5, li.getInterpolatedValue(13));
}
