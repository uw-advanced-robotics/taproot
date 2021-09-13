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

#ifndef LINEAR_INTERPOLATION_CONTIGUOUS_HPP_
#define LINEAR_INTERPOLATION_CONTIGUOUS_HPP_

#include <cstdint>
#include "contiguous_float.hpp"

namespace tap
{
namespace algorithms
{
/**
 * An object that is similar in every respect to the `LinearInterpolation`
 * object except that it uses `ContiguousFloat`'s instead.
 */
class LinearInterpolationContiguous
{
public:
    /**
     * @param[in] lowerBound Lower bound for linear interpolation contiguous float.
     * @param[in] upperBound Upper bound for linear interpolation contiguous float.
     */
    LinearInterpolationContiguous(float lowerBound, float upperBound);

    /**
     * Updates the interpolation using the newValue.
     *
     * @note only call this when you receive a new value (use remote rx
     *      counter to tell when there is new data from the remote, for
     *      example).
     * @param[in] newValue the new data used in the interpolation.
     * @param[in] currTime The time that this function was called.
     */
    void update(float newValue, uint32_t currTime);

    /**
     * Returns the current value, that is: \f$y\f$ in the equation
     * \f$y=slope\cdot (currTime - lastUpdateCallTime) + previousValue\f$.
     *
     * @note use a millisecond-resolution timer, e.g.
     *      tap::arch::clock::getTimeMilliseconds()
     * @param[in] currTime the current clock time, in ms.
     * @return the interpolated value.
     */
    float getInterpolatedValue(uint32_t currTime);

    /**
     * @param[in] initialValue The value to set the previous value to when resetting.
     * @param[in] initialTime The value to set the previous time to when resetting.
     */
    void reset(float initialValue, uint32_t initialTime);

private:
    uint32_t lastUpdateCallTime;  /// The previous timestamp from when update was called.
    ContiguousFloat previousValue;/// The previous data value.
    float slope;  /// The current slope, calculated using the previous and most current data.
};                // class LinearInterpolationContiguous

}  // namespace algorithms

}  // namespace tap

#endif  // LINEAR_INTERPOLATION_CONTIGUOUS_HPP_
