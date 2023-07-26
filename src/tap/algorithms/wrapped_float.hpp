/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_WRAPPED_FLOAT_HPP_
#define TAPROOT_WRAPPED_FLOAT_HPP_

#include <cmath>

#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

namespace tap
{
namespace algorithms
{
/**
 * Wraps a float to allow easy comparison and manipulation of sensor readings
 * that wrap (e.g. -180 to 180).
 *
 * For bounds 0 - 10, logically:
 *   - 10 + 1 == 1
 *   - 0 - 1 == 9
 *   - 0 == 10
 *
 * Credit to: https://github.com/Team488/SeriouslyCommonLib/blob/af2ce83a830299a8ab3773bec9b8ccc6ab
 *            5a3367/src/main/java/xbot/common/math/ContiguousDouble.java
 */
class WrappedFloat
{
public:
    WrappedFloat(const float value, const float lowerBound, const float upperBound);

    /** Overloaded Operators */

    /**
     * Adds a WrappedFloat to `this` WrappedFloat given they have the same lower and
     * upper bounds.
     *
     * @param[in] other: The WrappedFloat to be added to `this` WrappedFloat.
     * @throws: An assertion error if the two WrappedFloats have different lower and upper bounds.
     */
    void operator+= (const WrappedFloat& other);

    /**
     * Subtracts a WrappedFloat from `this` WrappedFloat given they have the same lower and
     * upper bounds.
     *
     * @param[in] other: The WrappedFloat to be subtracted from `this` WrappedFloat.
     * @throws: An assertion error if the two WrappedFloats have different lower and upper bounds.
     */
    void operator-= (const WrappedFloat& other);

    /**
     * Adds a given WrappedFloat to `this` WrappedFloat given they have the same lower and upper bounds, 
     * returning the resultant WrappedFloat.
     *
     * @param[in] other: The WrappedFloat to be added with `this` WrappedFloat.
     * @return: A new WrappedFloat with the additive value of `other` and `this`.
     * @throws: An assertion error if the two WrappedFloats have different lower and upper bounds.
     */
    WrappedFloat operator+ (const WrappedFloat& other) const;

    /**
     * Subtracts a given WrappedFloat from `this` WrappedFloat given they have the same lower and upper bounds,
     * returning the resultant WrappedFloat.
     *
     * @param[in] other: The WrappedFloat to be subtracted from `this` WrappedFloat.
     * @return: A new WrappedFloat with the subtractive value of `other` from `this`.
     * @throws: An assertion error if the two WrappedFloats have different lower and upper bounds.
     */
    WrappedFloat operator- (const WrappedFloat& other) const;

    /**
     * Scales WrappedFloat value relative to 0.
     */
    inline void operator*= (const float scale)
    {
        this->value_ = scale * this->value_;
        wrapValue();
    }

    /**
     * Inversely scales WrappedFloat value relative to 0.
    */
    inline void operator/= (const float scale)
    {
        this->value_ = this->value_ / scale;
        wrapValue();
    }

    /**
     * Shifts `this` WrappedFloat up in place by a given value.
     *
     * @param[in] shiftMagnitude: The amount to shift up by.
     */
    void shiftUpInPlace(float shiftMagnitude);

    /**
     * Shifts `this` WrappedFloat down in place by a given value.
     *
     * @param[in] shiftMagnitude: The amount to shift down by.
     */
    void shiftDownInPlace(float shiftMagnitude);

    /**
     * Shifts `this` WrappedFloat up by a given value, returning the resultant WrappedFloat.
     *
     * @param[in] shiftMagnitude: The amount to shift up by.
     * @return: A new WrappedFloat with the value of `this` shifted up.
     */
    WrappedFloat shiftUp(float shiftMagnitude) const;

    /**
     * Shifts `this` WrappedFloat down by a given value, returning the resultant WrappedFloat.
     *
     * @param[in] shiftMagnitude: The amount to shift down by.
     * @return: A new WrappedFloat with the value of `this` shifted down.
     */
    WrappedFloat shiftDown(float shiftMagnitude) const;

    /**
     * Computes the difference between another WrappedFloat and `this` WrappedFloat (other - this),
     * given they have the same lower and upper bounds.
     *
     * @param[in] other: The other value to compare against.
     * @return: A new WrappedFloat holding the computed difference.
     * @throws: An assertion error if the two WrappedFloats have different lower and upper bounds.
     */
    WrappedFloat minDifference(const WrappedFloat& other) const;

    /**
     * Shifts both bounds by the specified amount.
     *
     * @param[in] shiftMagnitude the amount to add to each bound.
     */
    void shiftBounds(const float shiftMagnitude);

    // Getters/Setters ----------------

    /**
     * Returns the wrapped value.
     */
    inline float getValue() const { return value_; };

    /**
     * Sets the wrapped value.
     */
    inline void setValue(const float newValue)
    {
        this->value_ = newValue; wrapValue();
    };

    /**
     * Returns the value's upper bound.
     */
    inline float getUpperBound() const { return upperBound_; };

    /**
     * Returns the value's lower bound.
     */
    inline float getLowerBound() const { return lowerBound_; };

    /**
     * Maximum value between floats representing bounds at which
     * they're considered to be "equal" for assertions.
    */
    static constexpr float EPSILON = 1E-8;

private:
    /**
     * The wrapped value.
     */
    float value_;

    /**
     * The lower bound to wrap around.
     */
    float lowerBound_;
    /**
     * The upper bound to wrap around.
     */
    float upperBound_;

    /**
     * Helper function for wrapping value within bounds.
     */
    void wrapValue();
};  // class WrappedFloat

/**
 * Represents an angle in radians.
 */
class Angle : public WrappedFloat
{
public:
    inline Angle(const float value) : WrappedFloat(value, -M_PI, M_PI)
    {
    };
};

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_WRAPPED_FLOAT_HPP_
