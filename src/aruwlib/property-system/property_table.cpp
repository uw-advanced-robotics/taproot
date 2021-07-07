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

#include "property_table.hpp"

#include <iostream>

namespace aruwlib::property
{
bool PropertyTable::addProperty(BasePropertyInterface *data)
{
    if (propertyTable.size() >= PROPERTY_TABLE_MAX_SIZE)
    {
        return false;
    }
    if (!data->getPropertyNameValid())
    {
        return false;
    }
    if (propertyTable.count(data->getPropertyName()))
    {
        return false;
    }
    propertyTable[data->getPropertyName()] = data;
    return true;
}

BasePropertyInterface *PropertyTable::removeProperty(const char *propertyName)
{
    if (propertyTable.count(propertyName) != 0)
    {
        BasePropertyInterface *removed = propertyTable[propertyName];
        propertyTable.erase(propertyName);
        return removed;
    }
    return nullptr;
}

const BasePropertyInterface *PropertyTable::getProperty(const char *propertyName)
{
    if (propertyTable.count(propertyName) != 0)
    {
        return propertyTable[propertyName];
    }
    return nullptr;
}
}  // namespace aruwlib::property
