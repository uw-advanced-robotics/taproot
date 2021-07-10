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

#ifndef BOOL_PROPERTY_HPP_
#define BOOL_PROPERTY_HPP_

#include <string>

#include "base_property.hpp"

namespace aruwlib::property
{
class BoolProperty : public BaseProperty<bool>
{
public:
    BoolProperty() : BaseProperty(), data(false) {}
    BoolProperty(bool data) : BaseProperty(), data(data) {}
    BoolProperty(bool data, const char* name) : BaseProperty(name), data(data) {}
    BoolProperty(const BoolProperty& other) = default;

    virtual ~BoolProperty() = default;

    void serializeData(uint8_t* arr) const override;
    uint16_t getSerializationArrSize() const override { return sizeof(bool); }
    std::string toString() const override { return data ? "true" : "false"; }
    void setProperty(bool data) override;

    explicit operator bool() const { return data; }
    BoolProperty& operator=(BoolProperty& p2) = default;
    BoolProperty& operator=(bool p2);
    BoolProperty& operator&=(BoolProperty& p1);
    BoolProperty& operator&=(bool p1);
    BoolProperty& operator|=(BoolProperty& p1);
    BoolProperty& operator|=(bool p1);

private:
    bool data;
};  // class BoolProperty

}  // namespace aruwlib::property

#endif  // BOOL_PROPERTY_HPP_
