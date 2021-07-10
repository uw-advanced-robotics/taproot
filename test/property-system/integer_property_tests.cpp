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

#include "aruwlib/property-system/integer_property.hpp"

using namespace aruwlib::property;

TEST(IntegerProperty, Default_constructor_constructs_zeroed_int32)
{
    IntegerProperty<int> p;

    EXPECT_EQ(0, p);
    EXPECT_EQ(nullptr, p.getPropertyName());
    EXPECT_EQ("0", p.toString());
}

TEST(IntegerProperty, Single_arg_constructor_allows_for_int32_specification)
{
    IntegerProperty<int> p(1234);

    EXPECT_EQ(1234, p);
    EXPECT_EQ(nullptr, p.getPropertyName());
    EXPECT_EQ(p.toString(), "1234");
}

TEST(IntegerProperty, Two_arg_constructor_allows_for_int32_and_name_specification)
{
    IntegerProperty<int> p(4321, "the property");

    EXPECT_EQ(4321, p);
    EXPECT_EQ("the property", p.getPropertyName());
    EXPECT_EQ("4321", p.toString());
}

TEST(IntegerProperty, Copy_constructor_copies_property_data_and_name)
{
    IntegerProperty<int> p1(4321, "the property");
    IntegerProperty<int> p2(p1);

    EXPECT_EQ(4321, p2);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("4321", p2.toString());
}

TEST(IntegerProperty, Equals_operator_copies_property_data_and_name)
{
    IntegerProperty<int> p1(1234, "the property");
    IntegerProperty<int> p2;

    p2 = p1;
    EXPECT_EQ(1234, p1);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("1234", p2.toString());
    p2 = 4321;
    EXPECT_EQ(4321, p2);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("4321", p2.toString());
}

TEST(IntegerProperty, Plus_operator_adds_data)
{
    IntegerProperty<int> p1(1);
    IntegerProperty<int> p2(2);

    EXPECT_EQ(3, p1 + p2);
    EXPECT_EQ(4, p1 + 3);
    EXPECT_EQ(4, 3 + p1);
    p1 += 3;
    EXPECT_EQ(4, p1);
    p1 += p2;
    EXPECT_EQ(6, p1);
}

TEST(IntegerProperty, Minus_operator_subtracts_data)
{
    IntegerProperty<int> p1(1);
    IntegerProperty<int> p2(2);

    EXPECT_EQ(-1, p1 - p2);
    EXPECT_EQ(-2, p1 - 3);
    EXPECT_EQ(2, 3 - p1);
    p1 -= 3;
    EXPECT_EQ(p1, -2);
    p1 -= p2;
    EXPECT_EQ(p1, -4);
}

TEST(IntegerProperty, Times_operator_multiplies_data)
{
    IntegerProperty<int> p1(2);
    IntegerProperty<int> p2(3);

    EXPECT_EQ(6, p1 * p2);
    EXPECT_EQ(8, p1 * 4);
    EXPECT_EQ(8, 4 * p1);
    p1 *= 4;
    EXPECT_EQ(8, p1);
    p1 *= p2;
    EXPECT_EQ(24, p1);
}

TEST(IntegerProperty, Divide_operator)
{
    IntegerProperty<int> p1(10);
    IntegerProperty<int> p2(2);

    EXPECT_EQ(5, p1 / p2);
    EXPECT_EQ(2, p1 / 5);
    EXPECT_EQ(10, 20 / p2);
    p1 /= 2;
    EXPECT_EQ(5, p1);
    p1 /= p2;
    EXPECT_EQ(2, p1);
}

TEST(IntegerProperty, getSerializationArrSize_returns_sizeof_int32)
{
    IntegerProperty<int> p(0x12345678, "the property");

    EXPECT_EQ(sizeof(int32_t), p.getSerializationArrSize());
}

TEST(IntegerProperty, serializeData)
{
    IntegerProperty<int> p(0x12345678, "the property");
    uint8_t *arr = new uint8_t[p.getSerializationArrSize()];

    p.serializeData(arr);
    EXPECT_EQ(0x12, arr[3]);
    EXPECT_EQ(0x34, arr[2]);
    EXPECT_EQ(0x56, arr[1]);
    EXPECT_EQ(0x78, arr[0]);

    delete[] arr;
}

TEST(IntegerProperty, setProperty_updates_data)
{
    IntegerProperty<int> p;

    p.setProperty(1);
    EXPECT_EQ(1, p);
    EXPECT_EQ("1", p.toString());
}

TEST(IntegerProperty, getPropertyId_unique_for_different_types)
{
    IntegerProperty<uint8_t> p1;
    IntegerProperty<uint16_t> p2;
    IntegerProperty<uint32_t> p3;
    IntegerProperty<uint64_t> p4;

    EXPECT_NE(p1.getPropertyId(), p2.getPropertyId());
    EXPECT_NE(p2.getPropertyId(), p3.getPropertyId());
    EXPECT_NE(p3.getPropertyId(), p4.getPropertyId());
    EXPECT_NE(p4.getPropertyId(), p1.getPropertyId());
}
