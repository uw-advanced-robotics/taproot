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

#include "tap/algorithms/linear_interpolation_predictor.hpp"

using namespace tap::algorithms;

TEST(LinearInterpolationPredictor, update_return_same_value_as_reset_if_time_before_reset_time_used)
{
    LinearInterpolationPredictor li;

    li.reset(5, 10);
    li.update(6, 5);
    EXPECT_FLOAT_EQ(5, li.getInterpolatedValue(100));
    li.update(3, 1);
    EXPECT_FLOAT_EQ(5, li.getInterpolatedValue(100));
}

TEST(LinearInterpolationPredictor, after_reset_return_reset_initial_val)
{
    LinearInterpolationPredictor li;

    li.reset(3, 5);
    EXPECT_FLOAT_EQ(3, li.getInterpolatedValue(100));
    li.reset(7, 1);
    EXPECT_FLOAT_EQ(7, li.getInterpolatedValue(100));
}

TEST(LinearInterpolationPredictor, normal_operation_no_wrapping)
{
    LinearInterpolationPredictor li;

    li.reset(3, 10);
    EXPECT_FLOAT_EQ(3, li.getInterpolatedValue(15));
    li.update(4, 20);
    EXPECT_FLOAT_EQ(4.5, li.getInterpolatedValue(25));
    li.update(6, 25);
    EXPECT_FLOAT_EQ(6.4, li.getInterpolatedValue(26));
    li.update(4, 26);
    EXPECT_FLOAT_EQ(2, li.getInterpolatedValue(27));
}
