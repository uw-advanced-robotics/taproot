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

#include "tap/algorithms/contiguous_float.hpp"

using namespace tap::algorithms;

TEST(ContiguousFloat, Basic_functionality)
{
    ContiguousFloat testInstance(5, 0, 10);
    EXPECT_EQ(5, testInstance.getValue());
}

TEST(ContiguousFloat, Wrapping_behavior)
{
    ContiguousFloat testInstance(-4, 0, 10);
    EXPECT_EQ(6, testInstance.getValue());

    testInstance.setValue(16);
    EXPECT_EQ(6, testInstance.getValue());

    testInstance.setValue(28);
    EXPECT_EQ(8, testInstance.getValue());
}

TEST(ContiguousFloat, Difference)
{
    ContiguousFloat testInstance(2, 0, 10);
    EXPECT_EQ(2, testInstance.difference(4));
    EXPECT_EQ(-1, testInstance.difference(11));

    testInstance.setValue(9);
    EXPECT_EQ(2, testInstance.difference(11));

    testInstance.setValue(10);
    EXPECT_EQ(1, testInstance.difference(1));
    testInstance.setValue(1);
    EXPECT_EQ(-1, testInstance.difference(10));
}

TEST(ContiguousFloat, Rotation_bounds)
{
    ContiguousFloat testInstance(150, -180, 180);

    EXPECT_EQ(40, testInstance.difference(190));
    EXPECT_EQ(40, testInstance.difference(-170));

    EXPECT_EQ(40, testInstance.difference(190));
    EXPECT_EQ(40, testInstance.difference(-170));

    testInstance.setValue(180);

    EXPECT_EQ(180, testInstance.getValue());
    EXPECT_EQ(0, testInstance.difference(-180));

    ContiguousFloat testInstance2(40, -180, 180);
    EXPECT_EQ(-140, testInstance2.difference(-100));
}

TEST(ContiguousFloat, Shifting_value)
{
    ContiguousFloat testInstance(150, -180, 180);

    testInstance.shiftValue(40);
    EXPECT_EQ(-170, testInstance.getValue());

    testInstance.shiftValue(40);
    EXPECT_EQ(-130, testInstance.getValue());

    testInstance.shiftValue(360);
    EXPECT_EQ(-130, testInstance.getValue());

    testInstance.shiftValue(0);
    EXPECT_EQ(-130, testInstance.getValue());
}

TEST(ContiguousFloat, Bad_bounds)
{
    ContiguousFloat testInstance(150, 180, -180);
    EXPECT_EQ(-180, testInstance.getLowerBound());
    EXPECT_EQ(180, testInstance.getUpperBound());
}

TEST(ContiguousFloat, shiftBounds_positive)
{
    ContiguousFloat testInstance(0, -100, 100);
    EXPECT_EQ(0, testInstance.getValue());
    testInstance.shiftBounds(200);
    EXPECT_EQ(200, testInstance.getValue());
}

TEST(ContiguousFloat, shiftBounds_negative)
{
    ContiguousFloat testInstance(10, -100, 100);
    EXPECT_EQ(10, testInstance.getValue());
    testInstance.shiftBounds(-200);
    EXPECT_EQ(-190, testInstance.getValue());
}

TEST(ContiguousFloat, setLowerBound_value_outside_new_bounds)
{
    ContiguousFloat testInstance(10, -100, 100);
    testInstance.setLowerBound(50);
    EXPECT_EQ(60, testInstance.getValue());
}

TEST(ContiguousFloat, setUpperBound_value_outside_new_bounds)
{
    ContiguousFloat testInstance(10, -100, 100);
    testInstance.setLowerBound(50);
    EXPECT_EQ(60, testInstance.getValue());
}

TEST(ContiguousFloat, limitVal_min_lt_max)
{
    ContiguousFloat testInstance(0, -100, 100);

    int status;
    EXPECT_EQ(0, ContiguousFloat::limitValue(testInstance, -10, 10, &status));
    EXPECT_EQ(0, status);

    testInstance.setValue(-20);
    EXPECT_EQ(-10, ContiguousFloat::limitValue(testInstance, -10, 10, &status));
    EXPECT_EQ(1, status);

    testInstance.setValue(20);
    EXPECT_EQ(10, ContiguousFloat::limitValue(testInstance, -10, 10, &status));
    EXPECT_EQ(2, status);
}

TEST(ContiguousFloat, limitVal_min_gt_max)
{
    ContiguousFloat testInstance(0, -100, 100);

    int status;

    EXPECT_EQ(5, ContiguousFloat::limitValue(testInstance, 5, -10, &status));
    EXPECT_EQ(1, status);

    EXPECT_EQ(-5, ContiguousFloat::limitValue(testInstance, 10, -5, &status));
    EXPECT_EQ(2, status);

    testInstance.setValue(20);
    EXPECT_EQ(20, ContiguousFloat::limitValue(testInstance, 10, -10, &status));
    EXPECT_EQ(0, status);
}
