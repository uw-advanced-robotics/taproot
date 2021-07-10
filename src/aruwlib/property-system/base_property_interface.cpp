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

#include "base_property_interface.hpp"

#include <cstring>

namespace aruwlib::property
{
uint16_t BasePropertyInterface::getFullSerializationSize() const
{
    return BASE_PROPERTY_HEADER_LENGTH + getSerializationArrSize();
}

void BasePropertyInterface::fullSerialization(uint8_t *arr) const
{
    if (arr == nullptr)
    {
        return;
    }
    arr[0] = getPropertyId();
    arr[1] = (getSerializationArrSize() >> 8) & 0xff;
    arr[2] = getSerializationArrSize() & 0xff;
    uint16_t propertyNameStrLen = strlen(propertyName);
    memcpy(arr + BASE_PROPERTY_HEADER_LENGTH, propertyName, propertyNameStrLen);
    serializeData(arr + BASE_PROPERTY_HEADER_LENGTH + propertyNameStrLen);
}

BasePropertyId BasePropertyIdCounter::count = 0U;

}  // namespace aruwlib::property
