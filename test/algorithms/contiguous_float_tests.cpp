/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "aruwlib/algorithms/contiguous_float.hpp"

TEST(ContiguousFloat, Basic_functionality)
{
    aruwlib::algorithms::ContiguousFloat testInstance(5, 0, 10);
    EXPECT_EQ(5, testInstance.getValue());
}

TEST(ContiguousFloat, Wrapping_behavior)
{
    aruwlib::algorithms::ContiguousFloat testInstance(-4, 0, 10);
    EXPECT_EQ(6, testInstance.getValue());

    testInstance.setValue(16);
    EXPECT_EQ(6, testInstance.getValue());

    testInstance.setValue(28);
    EXPECT_EQ(8, testInstance.getValue());
}

TEST(ContiguousFloat, Difference)
{
    aruwlib::algorithms::ContiguousFloat testInstance(2, 0, 10);
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
    aruwlib::algorithms::ContiguousFloat testInstance(150, -180, 180);

    EXPECT_EQ(40, testInstance.difference(190));
    EXPECT_EQ(40, testInstance.difference(-170));

    EXPECT_EQ(40, testInstance.difference(190));
    EXPECT_EQ(40, testInstance.difference(-170));

    testInstance.setValue(180);

    EXPECT_EQ(180, testInstance.getValue());
    EXPECT_EQ(0, testInstance.difference(-180));

    aruwlib::algorithms::ContiguousFloat testInstance2(40, -180, 180);
    EXPECT_EQ(-140, testInstance2.difference(-100));
}

TEST(ContiguousFloat, Shifting_value)
{
    aruwlib::algorithms::ContiguousFloat testInstance(150, -180, 180);

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
    aruwlib::algorithms::ContiguousFloat testInstance(150, 180, -180);
    EXPECT_EQ(-180, testInstance.getLowerBound());
    EXPECT_EQ(180, testInstance.getUpperBound());
}