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

#include "aruwlib/property-system/int32_property.hpp"

using aruwlib::property::Int32Property;

TEST(Int32Property, Default_constructor_constructs_zeroed_int32)
{
    Int32Property p;

    EXPECT_EQ(0, p);
    EXPECT_EQ(nullptr, p.getPropertyName());
    EXPECT_EQ("0", p.toString());
}

TEST(Int32Property, Single_arg_constructor_allows_for_int32_specification)
{
    Int32Property p(1234);

    EXPECT_EQ(1234, p);
    EXPECT_EQ(nullptr, p.getPropertyName());
    EXPECT_EQ(p.toString(), "1234");
}

TEST(Int32Property, Two_arg_constructor_allows_for_int32_and_name_specification)
{
    Int32Property p(4321, "the property");

    EXPECT_EQ(4321, p);
    EXPECT_EQ("the property", p.getPropertyName());
    EXPECT_EQ("4321", p.toString());
}

TEST(Int32Property, Copy_constructor_copies_property_data_and_name)
{
    Int32Property p1(4321, "the property");
    Int32Property p2(p1);

    EXPECT_EQ(4321, p2);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("4321", p2.toString());
}

TEST(Int32Property, Equals_operator_copies_property_data_and_name)
{
    Int32Property p1(1234, "the property");
    Int32Property p2;

    p2 = p1;
    EXPECT_EQ(1234, p1);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("1234", p2.toString());
    p2 = 4321;
    EXPECT_EQ(4321, p2);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("4321", p2.toString());
}

TEST(Int32Property, Plus_operator_adds_data)
{
    Int32Property p1(1);
    Int32Property p2(2);

    EXPECT_EQ(3, p1 + p2);
    EXPECT_EQ(4, p1 + 3);
    EXPECT_EQ(4, 3 + p1);
    p1 += 3;
    EXPECT_EQ(4, p1);
    p1 += p2;
    EXPECT_EQ(6, p1);
}

TEST(Int32Property, Minus_operator_subtracts_data)
{
    Int32Property p1(1);
    Int32Property p2(2);

    EXPECT_EQ(-1, p1 - p2);
    EXPECT_EQ(-2, p1 - 3);
    EXPECT_EQ(2, 3 - p1);
    p1 -= 3;
    EXPECT_EQ(p1, -2);
    p1 -= p2;
    EXPECT_EQ(p1, -4);
}

TEST(Int32Property, Times_operator_multiplies_data)
{
    Int32Property p1(2);
    Int32Property p2(3);

    EXPECT_EQ(6, p1 * p2);
    EXPECT_EQ(8, p1 * 4);
    EXPECT_EQ(8, 4 * p1);
    p1 *= 4;
    EXPECT_EQ(8, p1);
    p1 *= p2;
    EXPECT_EQ(24, p1);
}

TEST(Int32Property, Divide_operator)
{
    Int32Property p1(10);
    Int32Property p2(2);

    EXPECT_EQ(5, p1 / p2);
    EXPECT_EQ(2, p1 / 5);
    EXPECT_EQ(10, 20 / p2);
    p1 /= 2;
    EXPECT_EQ(5, p1);
    p1 /= p2;
    EXPECT_EQ(2, p1);
}

TEST(Int32Property, getSerializationArrSize_returns_sizeof_int32)
{
    Int32Property p(0x12345678, "the property");

    EXPECT_EQ(sizeof(int32_t), p.getSerializationArrSize());
}

TEST(Int32Property, serializeData)
{
    Int32Property p(0x12345678, "the property");
    uint8_t *arr = new uint8_t[p.getSerializationArrSize()];

    p.serializeData(arr);
    EXPECT_EQ(0x12, arr[3]);
    EXPECT_EQ(0x34, arr[2]);
    EXPECT_EQ(0x56, arr[1]);
    EXPECT_EQ(0x78, arr[0]);

    delete[] arr;
}

TEST(Int32Property, setProperty_updates_data)
{
    Int32Property p;

    p.setProperty(1);
    EXPECT_EQ(1, p);
    EXPECT_EQ("1", p.toString());
}
