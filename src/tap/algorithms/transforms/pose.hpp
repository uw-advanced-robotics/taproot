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

#ifndef TAPROOT_POSE_HPP_
#define TAPROOT_POSE_HPP_

#include "position.hpp"

namespace tap::algorithms::transforms
{

template <typename FRAME>
struct Pose
{
    Pose(float x, float y, float z, float A, float B, float C)
    : position(x, y, z), orientation(rotationMatrix(A, B, C)) {}

    Pose(CMSISMat<3, 1> position, CMSISMat<3, 3> orientation)
    : position(position)
    {
        this->orientation = std::move(orientation);
    }

    inline float getA() { return asinf(orientation.data[7]/cosf(getB())); }

    inline float getB() { return asinf(-orientation.data[6]); }

    inline float getC() { return asinf(orientation.data[3]/cosf(getB())); }

    Position<FRAME> position.
    CMSISMat<3, 3> orientation;
};  // struct Pose
}   // namespace tap::algorithms::transforms

#endif  // TAP_POSE_HPP_
