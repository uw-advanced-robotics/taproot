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

#ifndef PROPERTY_TABLE_HPP_
#define PROPERTY_TABLE_HPP_

#include <map>
#include <string>

#include "base_property.hpp"

namespace aruwlib::property
{
/**
 * Stores a map of property names to property types. This table has basic
 * insertion, removal, and interaction features that one can utilize.
 */
class PropertyTable
{
public:
    /**
     * The max size of the table.
     */
    static constexpr int PROPERTY_TABLE_MAX_SIZE = 512;

    PropertyTable() = default;

    PropertyTable(const PropertyTable &) = default;

    PropertyTable &operator=(const PropertyTable &) = default;

    /**
     * Check if number of properties in PropertyTable reaches maximum.
     *
     * @return `true` if the PropertyTable is full, `false` otherwise.
     */
    bool isFull() const { return propertyTable.size() == PROPERTY_TABLE_MAX_SIZE; }

    /**
     * @return The number of elements in the table.
     */
    int getSize() const { return propertyTable.size(); }

    /**
     * Adds `data` to the PropertyTable.
     *
     * @param[in] data Pointer to data to be managed.
     * @note The property table is not responsible for freeing memory associated
     *      with `data`.
     */
    bool addProperty(BasePropertyInterface *data);

    /**
     * Removes the property from the table by name. The removed property is returned.
     *
     * @param[in] propertyName The name of the property to be removed.
     */
    BasePropertyInterface *removeProperty(const char *propertyName);

    /**
     * Returns a const pointer to the property associated with the propertyName.
     *
     * @param[in] propertyName The name of the property to get.
     * @return The property associated with propertyName. `nullptr` if the property
     *      is not in the table.
     */
    const BasePropertyInterface *getProperty(const char *propertyName);

    /**
     * Sets the data of the property associated to the `propertyName` in the table to the
     * passed in data.
     *
     * @note a `reinterpret_cast` is used when setting the property's data. Be sure the
     *      data type is correct.
     * @tparam The data type of the property to be set.
     * @tparam The type of property that the data corresponds to derived from the
     * BasePropertyInterface.
     * @param[in] propertyName The name of the property to be set.
     * @param[in] data The data that the property will be set to.
     * @return `true` if the property was set successfully, `false` otherwise.
     */
    template <typename T>
    bool setProperty(const char *propertyName, T data)
    {
        if (propertyTable.count(propertyName) != 0)
        {
            BasePropertyInterface *property = propertyTable[propertyName];
            if (property == nullptr)
            {
                return false;
            }
            BaseProperty<T> *derivedProperty = dynamic_cast<BaseProperty<T> *>(property);
            if (derivedProperty == nullptr)
            {
                return false;
            }
            derivedProperty->setProperty(data);
            return true;
        }
        return false;
    }

    /**
     * @return A `const_iterator` to the beginning of the property table. This allows you to view
     *      table entries but not change them.
     */
    std::map<const char *, BasePropertyInterface *>::const_iterator getPropertyTableBeginning()
        const
    {
        return propertyTable.begin();
    }

    /**
     * @return A `const_iterator` to the end of the property table. This allows you to know when
     *      you have traversed the entire property table if using `getPropertyTableBeginning`.
     */
    std::map<const char *, BasePropertyInterface *>::const_iterator getPropertyTableEnd() const
    {
        return propertyTable.end();
    }

private:
    struct CharComparator
    {
        bool operator()(const char *a, const char *b) const { return std::strcmp(a, b) < 0; }
    };

    std::map<const char *, BasePropertyInterface *, CharComparator> propertyTable;
};  // class PropertyTable

}  // namespace aruwlib::property

#endif  // PROPERTY_TABLE_HPP_
