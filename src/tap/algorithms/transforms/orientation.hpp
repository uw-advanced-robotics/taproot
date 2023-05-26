/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_ORIENTATION_HPP_
#define TAPROOT_ORIENTATION_HPP_

#include "tap/algorithms/cmsis_mat.hpp"

namespace tap::algorithms::transforms
{

template <typename FRAME>
class Orientation
{
public:
    inline Orientation(const float roll, const float pitch, const float yaw)
    {
        coordinates = std::move(CMSISMat<3,3>({
            cosf(yaw) * cosf(pitch),
            (cosf(yaw) * sinf(pitch) * sinf(roll)) - (sinf(yaw) * cosf(roll)),
            (cosf(yaw) * sinf(pitch) * cosf(roll)) + sinf(yaw) * sinf(roll),
            sinf(yaw) * cosf(pitch),
            sinf(yaw) * sinf(pitch) * sinf(roll) + cosf(yaw) * cosf(roll),
            sinf(yaw) * sinf(pitch) * cosf(roll) - cosf(yaw) * sinf(roll),
            -sinf(pitch),
            cosf(pitch) * sinf(roll),
            cosf(pitch) * cosf(roll)
        }));
    };

    // TODO: should we consider the possibility of gimbal lock?
    // TODO: return angle objects
    /**
     * Returns roll as values between [-pi, +pi].
     * 
     * If pitch is completely vertical (-pi / 2 or pi / 2) then roll and yaw are gimbal-locked. In this case, roll is taken to be 0.
     */
    inline float roll() const
    {
        return atan2(coordinates.data[7], coordinates.data[8]);
    }

    inline float pitch() const { return asinf(-coordinates.data[6]); }

    inline float yaw() const { return atan2(coordinates.data[3], coordinates.data[0]); }

private:
    CMSISMat<3, 3> coordinates;
};
}

#endif