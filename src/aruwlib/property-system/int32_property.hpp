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

#ifndef INT32_PROPERTY_HPP_
#define INT32_PROPERTY_HPP_

#include <cinttypes>
#include <string>

#include "base_property.hpp"

namespace aruwlib::property
{
class Int32Property : public BaseProperty<int32_t>
{
public:
    Int32Property() : BaseProperty(), data(0) {}
    Int32Property(int32_t data) : BaseProperty(), data(data) {}
    Int32Property(int32_t data, const char* name) : BaseProperty(name), data(data) {}
    Int32Property(const Int32Property& other) = default;

    virtual ~Int32Property() = default;

    void serializeData(uint8_t* arr) const override;
    uint16_t getSerializationArrSize() const override { return sizeof(int32_t); }
    PROPERTY_TYPE_ID getPropertyType() const override { return PROPERTY_TYPE_ID::INT32; }
    std::string toString() const override { return std::to_string(data).c_str(); }
    void setProperty(int32_t data) override;

    operator int32_t() const { return data; }
    Int32Property& operator=(Int32Property& other) = default;
    Int32Property& operator=(int32_t other);
    Int32Property& operator+=(Int32Property& other);
    Int32Property& operator+=(int32_t other);
    Int32Property& operator-=(Int32Property& other);
    Int32Property& operator-=(int32_t other);
    Int32Property& operator*=(Int32Property& other);
    Int32Property& operator*=(int32_t other);
    Int32Property& operator/=(Int32Property& other);
    Int32Property& operator/=(int32_t other);

private:
    int32_t data;
};  // class Int32Property

}  // namespace aruwlib::property

#endif  // INT32_PROPERTY_HPP_
