/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_QUANTITY_HPP_
#define TAPROOT_QUANTITY_HPP_
#include <ratio>
#include <type_traits>
using std::ratio, std::ratio_add, std::ratio_subtract, std::ratio_multiply, std::ratio_divide,
    std::ratio_equal;
namespace tap::units
{
/**
 * @brief A class representing a scalar quantity with unit dimensions.
 */
template <
    typename Time = ratio<0>,
    typename Length = ratio<0>,
    typename Mass = ratio<0>,
    typename Current = ratio<0>,
    typename Temperature = ratio<0>,
    typename Angle = ratio<0>,
    typename Frame = ratio<0>>
class Quantity
{
protected:
    float value;

public:
    // Convenience labels representing the dimensions, for use in template metaprogramming

    /// The time dimension of the quantity, with a base unit of seconds
    typedef Time time;
    /// The length dimension of the quantity, with a base unit of meters
    typedef Length length;
    /// The mass dimension of the quantity, with a base unit of kilograms
    typedef Mass mass;
    /// The current dimension of the quantity, with a base unit of amperes
    typedef Current current;
    /// The temperature dimension of the quantity, with a base unit of kelvin
    typedef Temperature temperature;
    /// The angle dimension of the quantity, with a base unit of radians
    typedef Angle angle;
    /// The "context frame" of the quantity, that can be user-defined.
    typedef Frame frame;

    /**
     * @brief convenience label. Represents an isomorphic unit (equal dimensions)
     */
    using Self = Quantity<Time, Length, Mass, Current, Temperature, Angle, Frame>;

    // Constructors
    /**
     * @brief Construct a new Quantity object
     * @param value The new value of the quantity, in its base unit
     */
    constexpr Quantity(float value) : value(value) {}
    /**
     * @brief Construct a new Quantity object. Default constructor, initializes value to 0
     */
    constexpr Quantity() : value(0) {}
    /**
     * @brief Construct a new Quantity object
     * @param other The other quantity to copy
     */
    constexpr Quantity(const Quantity other) : value(other.value) {}

    /**
     * @brief Returns the value of the quantity in its base unit
     * @return The value of the quantity
     */
    constexpr float valueOf() const { return value; }

    /**
     * @brief Returns the value of the quantity converted to another unit
     * @param other The other unit to convert to
     */
    constexpr float convertTo(const Self unit) const { return value / unit.value; }

    // Operators

    /**
     * @brief Adds another quantity to this one
     * @param other The right hand addend
     */
    constexpr void operator+=(const Self other) { value += other.value; }

    /**
     * @brief Subtracts another quantity from this one
     * @param other The right hand minuend
     */
    constexpr void operator-=(const Self other) { value -= other.value; }

    /**
     * @brief Multiplies this quantity by a unitless factor
     * @param multiple The factor to multiply by
     */
    constexpr void operator*=(const float multiple) { value *= multiple; }

    /**
     * @brief Divides this quantity by a unitless factor
     * @param scalar The factor to divide by
     */
    constexpr void operator/=(const float dividend) { value /= dividend; }

    /**
     * @brief Assign a raw numerical value to this quantity. Only works if this quantity is
     * dimensionless.
     */
    constexpr void operator=(const float other)  // TODO: how much do we actually care about this?
    {
        static_assert(
            std::convertable_to<Self, Quantity<>>,
            "Cannot assign a raw float to a non-dimensionless quanity");
        value = other;
    }
};
}  // namespace tap::units
#endif  // TAPROOT_QUANTITY_HPP_