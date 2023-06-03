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

#ifndef TAPROOT_ROTATION_HPP_
#define TAPROOT_ROTATION_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/euler_angles.hpp"
#include "frame.hpp"

namespace tap::algorithms::transforms
{

template <const Frame& FRAME>
class Rotation
{
public:
    inline Rotation(const float roll, const float pitch, const float yaw)
        : coordinates_(fromEulerAngles(roll, pitch, yaw))
    {
    };

    inline Rotation(CMSISMat<3, 3> matrix)
        : coordinates_(std::move(matrix))
    {
    }

    // TODO: sort out copy constructor and copy assignment because default directly copies cmsismat

    // TODO: should we consider the possibility of gimbal lock?
    // TODO: return angle objects
    /**
     * Returns roll as values between [-pi, +pi].
     * 
     * @warning if pitch is completely vertical (-pi / 2 or pi / 2) then roll and yaw are gimbal-locked
     */
    inline float roll() const { return atan2(coordinates_.data[7], coordinates_.data[8]); }

    /**
     * Returns pitch as values between [-pi, +pi].
    */
    inline float pitch() const { return asinf(-coordinates_.data[6]); }

    /**
     * Returns yaw as values between [-pi/2, +pi/2].
     * 
     * @warning if pitch is completely vertical then roll and yaw are gimbal-locked.
    */
    inline float yaw() const { return atan2(coordinates_.data[3], coordinates_.data[0]); }

private:
    CMSISMat<3, 1> axisAngle_;
};
}

#endif