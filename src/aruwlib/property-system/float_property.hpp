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

#ifndef FLOAT_PROPERTY_HPP_
#define FLOAT_PROPERTY_HPP_

#include <cinttypes>
#include <string>

#include "base_property.hpp"

namespace aruwlib::property
{
class FloatProperty : public BaseProperty<float>
{
public:
    FloatProperty() : data(0.0f) {}
    FloatProperty(float data) : data(data) {}
    FloatProperty(float data, const char* name) : BaseProperty(name), data(data) {}
    FloatProperty(const FloatProperty& other) = default;

    virtual ~FloatProperty() = default;

    void serializeData(uint8_t* arr) const override;
    uint16_t getSerializationArrSize() const override { return sizeof(float); }
    std::string toString() const override;
    void setProperty(float data) override;

    operator float() const { return data; }
    FloatProperty& operator=(FloatProperty& other) = default;
    FloatProperty& operator=(float other);
    FloatProperty& operator+=(FloatProperty& other);
    FloatProperty& operator+=(float other);
    FloatProperty& operator-=(FloatProperty& other);
    FloatProperty& operator-=(float other);
    FloatProperty& operator*=(FloatProperty& other);
    FloatProperty& operator*=(float other);
    FloatProperty& operator/=(FloatProperty& other);
    FloatProperty& operator/=(float other);

private:
    float data;
};

}  // namespace aruwlib::property

#endif  // FLOAT_PROPERTY_HPP_
