/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/units/frame.hpp"

namespace tap::units
{
using std::ratio, std::ratio_add, std::ratio_subtract, std::ratio_multiply, std::ratio_divide,
    std::ratio_equal;

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
    typename Frame = DefaultFrame>
class Quantity
{
protected:
    float value;

    static_assert(
        std::ratio_not_equal_v<typename FrameConvert<DefaultFrame, Frame>::factor, ratio<0, 1>>,
        "Default frame conversion not defined");

    static_assert(
        std::ratio_equal_v<typename FrameConvert<Frame, Frame>::factor, ratio<1, 1>>,
        "Frame type given without valid self-conversion");

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
    /// The frame of reference of the quantity
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
    explicit constexpr Quantity(float value) : value(value) {}

    /**
     * @brief Construct a new Quantity object. Default constructor, initializes value to 0
     */
    explicit constexpr Quantity() : value(0) {}

    /**
     * @brief Construct a new Quantity object
     * @param other The other quantity to copy
     */
    constexpr Quantity(const Self& other) : value(other.value) {}

    /**
     * @brief Returns the value of the quantity in its base unit
     * @return The value of the quantity
     */
    constexpr float internalValue() const { return value; }

    // Operators

    /**
     * @brief Assign a new value to this quantity
     * @param other The new value
     */
    constexpr void operator=(const Self other) { value = other.value; }
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

    template <typename F>
    constexpr Quantity<Time, Length, Mass, Current, Temperature, Angle, F>
    convertFrame() requires std::
        ratio_not_equal_v<typename FrameConvert<DefaultFrame, F>::factor, ratio<0, 1>>
    {
        return Quantity<Time, Length, Mass, Current, Temperature, Angle, F>(
            value * ((float)FrameConvert<Frame, F>::factor::num /
                     (float)FrameConvert<Frame, F>::factor::den));
    }
};

};      // namespace tap::units
#endif  // TAPROOT_QUANTITY_HPP_