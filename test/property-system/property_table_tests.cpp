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

#include <iostream>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "aruwlib/property-system/int32_property.hpp"
#include "aruwlib/property-system/property_table.hpp"

using aruwlib::property::BaseProperty;
using aruwlib::property::BasePropertyInterface;
using aruwlib::property::Int32Property;
using aruwlib::property::PropertyTable;

TEST(
    PropertyTable,
    addProperty_successful_using_Int32Property_getProperty_returns_pointer_to_added_property)
{
    PropertyTable ptable;
    Int32Property property(3, "cool property");

    EXPECT_TRUE(ptable.addProperty(&property));
    const Int32Property *propertyPtr =
        dynamic_cast<const Int32Property *>(ptable.getProperty("cool property"));
    EXPECT_NE(nullptr, propertyPtr);
    EXPECT_EQ(propertyPtr, &property);
}

TEST(PropertyTable, removeProperty_successfully_removes_Int32Property_if_in_table)
{
    PropertyTable ptable;
    Int32Property property(3, "cool property");

    EXPECT_TRUE(ptable.addProperty(&property));
    const Int32Property *propertyPtr =
        dynamic_cast<const Int32Property *>(ptable.removeProperty("cool property"));
    EXPECT_NE(nullptr, propertyPtr);
    EXPECT_EQ(propertyPtr, &property);
}

TEST(PropertyTable, setProperty)
{
    PropertyTable ptable;
    Int32Property p(42, "p");

    ptable.addProperty(&p);
    const Int32Property *ppointer = dynamic_cast<const Int32Property *>(ptable.getProperty("p"));
    EXPECT_EQ(ppointer, &p);
    EXPECT_EQ(42, *ppointer);
    EXPECT_TRUE(ptable.setProperty<int32_t>("p", 41));
    EXPECT_EQ(41, *ppointer);
    EXPECT_EQ(41, p);
}

TEST(PropertyTable, isFull_getSize_reflect_number_of_properties_in_table)
{
    PropertyTable ptable;
    char *numberStrs[PropertyTable::PROPERTY_TABLE_MAX_SIZE];
    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        EXPECT_FALSE(ptable.isFull());
        EXPECT_EQ(i, ptable.getSize());

        std::string name = std::to_string(i);
        numberStrs[i] = new char[name.size() + 1];
        strcpy(numberStrs[i], name.c_str());

        Int32Property *p = new Int32Property(i, numberStrs[i]);
        ptable.addProperty(p);
    }

    Int32Property property(PropertyTable::PROPERTY_TABLE_MAX_SIZE, "extra property");
    EXPECT_FALSE(ptable.addProperty(&property));

    EXPECT_TRUE(ptable.isFull());
    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        BasePropertyInterface *p = ptable.removeProperty(numberStrs[i]);
        delete[] numberStrs[i];
        delete p;
    }
}

TEST(PropertyTable, getProperty_works_with_c_strs)
{
    PropertyTable ptable;
    const char *name1 = "hi";
    const char *name2 = "hi";
    Int32Property p(10, name1);
    ptable.addProperty(&p);
    const BasePropertyInterface *property = ptable.getProperty(name2);
    EXPECT_EQ(&p, dynamic_cast<const Int32Property *>(property));
}

TEST(
    PropertyTable,
    getPropertyTableBeginning_getPropertyTableEnd_allows_for_proper_iteration_through_elements)
{
    PropertyTable ptable;
    std::set<const char *> ptableContents;
    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        EXPECT_FALSE(ptable.isFull());
        EXPECT_EQ(i, ptable.getSize());

        char *name = new char[std::to_string(i).size() + 1];
        strcpy(name, std::to_string(i).c_str());
        ptableContents.emplace(name);

        Int32Property *p = new Int32Property(i, name);
        ptable.addProperty(p);
    }

    int numMatching = 0;
    for (auto iter = ptable.getPropertyTableBeginning(); iter != ptable.getPropertyTableEnd();
         iter++)
    {
        EXPECT_EQ(1, ptableContents.count(iter->first));
        numMatching++;
    }
    EXPECT_EQ(PropertyTable::PROPERTY_TABLE_MAX_SIZE, numMatching);

    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        BasePropertyInterface *p = ptable.removeProperty(std::to_string(i).c_str());
        delete p;
    }

    EXPECT_EQ(0, ptable.getSize());

    std::for_each(ptableContents.begin(), ptableContents.end(), [](const char *name) {
        delete[] name;
    });
    ptableContents.clear();
}
