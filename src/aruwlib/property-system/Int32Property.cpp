/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "int32_property.hpp"

namespace aruwlib::property
{
void Int32Property::serializeData(uint8_t* arr) const
{
    if (arr == nullptr)
    {
        return;
    }
    memcpy(arr, &data, sizeof(int32_t));
}

void Int32Property::setProperty(int32_t data) { this->data = data; }

Int32Property& Int32Property::operator=(int32_t other)
{
    data = other;
    return *this;
}

Int32Property& Int32Property::operator+=(Int32Property& other)
{
    data += other.data;
    return *this;
}

Int32Property& Int32Property::operator+=(int32_t other)
{
    data += other;
    return *this;
}

Int32Property& Int32Property::operator-=(Int32Property& other)
{
    data -= other.data;
    return *this;
}

Int32Property& Int32Property::operator-=(int32_t other)
{
    data -= other;
    return *this;
}

Int32Property& Int32Property::operator*=(Int32Property& other)
{
    data *= other.data;
    return *this;
}

Int32Property& Int32Property::operator*=(int32_t other)
{
    data *= other;
    return *this;
}

Int32Property& Int32Property::operator/=(Int32Property& other)
{
    data /= other.data;
    return *this;
}

Int32Property& Int32Property::operator/=(int32_t other)
{
    data /= other;
    return *this;
}

}  // namespace aruwlib
