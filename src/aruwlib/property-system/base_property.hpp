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

#ifndef BASE_PROPERTY_HPP_
#define BASE_PROPERTY_HPP_

#include <cstring>

#include "base_property_interface.hpp"

namespace aruwlib::property
{
template <typename T>
class BaseProperty : public BasePropertyInterface
{
public:
    BaseProperty() : BasePropertyInterface() {}

    BaseProperty(const char *name) : BasePropertyInterface(name) {}

    virtual ~BaseProperty() = default;

    /**
     * Sets the data stored in the property to the passed in `data`.
     *
     * @param[in] data A pointer to the data to be set.
     * @return `true` if the property has been set properly, `false` otherwise.
     */
    virtual void setProperty(T data) = 0;

    BasePropertyId getPropertyId() const override { return STATIC_BASE_PROPERTY_ID; }

private:
    static const BasePropertyId STATIC_BASE_PROPERTY_ID;
};  // class BaseProperty

template <typename T>
const BasePropertyId BaseProperty<T>::STATIC_BASE_PROPERTY_ID = BasePropertyIdCounter::get<T>();

}  // namespace aruwlib::property

#endif  // BASE_PROPERTY_HPP_
