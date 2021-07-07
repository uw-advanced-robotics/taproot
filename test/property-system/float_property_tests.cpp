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

#include "aruwlib/property-system/float_property.hpp"

using aruwlib::property::FloatProperty;

TEST(FloatProperty, Default_constructor_constructs_zeroed_float)
{
    FloatProperty p;

    EXPECT_EQ(0, p);
    EXPECT_EQ(nullptr, p.getPropertyName());
    EXPECT_EQ("0", p.toString());
}

TEST(FloatProperty, Single_arg_constructor_allows_for_float_specification)
{
    FloatProperty p(42.25f);

    EXPECT_EQ(42.25f, p);
    EXPECT_EQ(nullptr, p.getPropertyName());
    EXPECT_EQ(p.toString(), "42.25");
}

TEST(FloatProperty, Two_arg_constructor_allows_for_float_and_name_specification)
{
    FloatProperty p(43.56, "the property");

    EXPECT_EQ(43.56f, p);
    EXPECT_EQ("the property", p.getPropertyName());
    EXPECT_EQ("43.56", p.toString());
}

TEST(FloatProperty, Copy_constructor_copies_property_data_and_name)
{
    FloatProperty p1(43.56f, "the property");
    FloatProperty p2(p1);

    EXPECT_EQ(43.56f, p2);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("43.56", p2.toString());
}

TEST(FloatProperty, Equals_operator_copies_property_data_and_name)
{
    FloatProperty p1(42.25f, "the property");
    FloatProperty p2;

    p2 = p1;
    EXPECT_EQ(42.25f, p1);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("42.25", p2.toString());
    p2 = 43.56f;
    EXPECT_EQ(43.56f, p2);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("43.56", p2.toString());
}

TEST(FloatProperty, Plus_operator_adds_data)
{
    FloatProperty p1(1);
    FloatProperty p2(2);

    EXPECT_EQ(3, p1 + p2);
    EXPECT_EQ(4, p1 + 3);
    EXPECT_EQ(4, 3 + p1);
    p1 += 3;
    EXPECT_EQ(4, p1);
    p1 += p2;
    EXPECT_EQ(6, p1);
}

TEST(FloatProperty, Minus_operator_subtracts_data)
{
    FloatProperty p1(1);
    FloatProperty p2(2);

    EXPECT_EQ(-1, p1 - p2);
    EXPECT_EQ(-2, p1 - 3);
    EXPECT_EQ(2, 3 - p1);
    p1 -= 3;
    EXPECT_EQ(p1, -2);
    p1 -= p2;
    EXPECT_EQ(p1, -4);
}

TEST(FloatProperty, Times_operator_multiplies_data)
{
    FloatProperty p1(2);
    FloatProperty p2(3);

    EXPECT_EQ(6, p1 * p2);
    EXPECT_EQ(8, p1 * 4);
    EXPECT_EQ(8, 4 * p1);
    p1 *= 4;
    EXPECT_EQ(8, p1);
    p1 *= p2;
    EXPECT_EQ(24, p1);
}

TEST(FloatProperty, Divide_operator)
{
    FloatProperty p1(10);
    FloatProperty p2(2);

    EXPECT_EQ(5, p1 / p2);
    EXPECT_EQ(2, p1 / 5);
    EXPECT_EQ(10, 20 / p2);
    p1 /= 2;
    EXPECT_EQ(5, p1);
    p1 /= p2;
    EXPECT_EQ(2.5f, p1);
}

TEST(FloatProperty, getSerializationArrSize_returns_sizeof_float)
{
    FloatProperty p;

    EXPECT_EQ(sizeof(float), p.getSerializationArrSize());
}

TEST(FloatProperty, serializeData)
{
    const float dat = 42.4f;
    uint8_t datArr[sizeof(float)];
    memcpy(&datArr, &dat, sizeof(float));
    FloatProperty p(dat, "the property");
    uint8_t *arr = new uint8_t[p.getSerializationArrSize()];

    p.serializeData(arr);
    EXPECT_EQ(datArr[0], arr[0]);
    EXPECT_EQ(datArr[1], arr[1]);
    EXPECT_EQ(datArr[2], arr[2]);
    EXPECT_EQ(datArr[3], arr[3]);

    delete[] arr;
}

TEST(FloatProperty, setProperty_updates_data)
{
    FloatProperty p;

    p.setProperty(1);
    EXPECT_EQ(1, p);
    EXPECT_EQ("1", p.toString());
}
