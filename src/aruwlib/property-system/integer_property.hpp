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

#ifndef NUMBER_PROPERTY_HPP_
#define NUMBER_PROPERTY_HPP_

#include <cinttypes>
#include <string>

#include "aruwlib/architecture/endianness_wrappers.hpp"

#include "base_property.hpp"

namespace aruwlib::property
{
template <typename T>
class IntegerProperty : public BaseProperty<T>
{
public:
    IntegerProperty() : BaseProperty<T>(), data(0) {}
    IntegerProperty(T data) : BaseProperty<T>(), data(data) {}
    IntegerProperty(T data, const char* name) : BaseProperty<T>(name), data(data) {}
    IntegerProperty(const IntegerProperty& other) = default;

    virtual ~IntegerProperty() = default;

    void serializeData(uint8_t* arr) const override { arch::convertToLittleEndian(data, arr); }

    uint16_t getSerializationArrSize() const override { return sizeof(T); }

    std::string toString() const override { return std::to_string(data).c_str(); }

    void setProperty(T data) override { this->data = data; }

    operator T() const { return data; }
    IntegerProperty<T>& operator=(IntegerProperty<T>& other) = default;
    IntegerProperty<T>& operator=(T other)
    {
        data = other;
        return *this;
    }
    IntegerProperty& operator+=(IntegerProperty<T>& other)
    {
        data += other.data;
        return *this;
    }
    IntegerProperty& operator+=(T other)
    {
        data += other;
        return *this;
    }
    IntegerProperty& operator-=(IntegerProperty<T>& other)
    {
        data -= other.data;
        return *this;
    }
    IntegerProperty& operator-=(T other)
    {
        data -= other;
        return *this;
    }
    IntegerProperty& operator*=(IntegerProperty<T>& other)
    {
        data *= other.data;
        return *this;
    }
    IntegerProperty& operator*=(T other)
    {
        data *= other;
        return *this;
    }
    IntegerProperty& operator/=(IntegerProperty<T>& other)
    {
        data /= other.data;
        return *this;
    }
    IntegerProperty& operator/=(T other)
    {
        data /= other;
        return *this;
    }

private:
    T data;
};  // class IntegerProperty

}  // namespace aruwlib::property

#endif  // NUMBER_PROPERTY_HPP_
