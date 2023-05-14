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
        : position(x, y, z),
          orientation({
            cosf(C) * cosf(B),
            (cosf(C) * sinf(B) * sinf(A)) - (sinf(C) * cosf(A)),
            (cosf(C) * sinf(B) * cosf(A)) + sinf(C) * sinf(A),
            sinf(C) * cosf(B),
            sinf(C) * sinf(B) * sinf(A) + cosf(C) * cosf(A),
            sinf(C) * sinf(B) * cosf(A) - cosf(C) * sinf(A),
            -sinf(B),
            cosf(B) * sinf(A),
            cosf(B) * cosf(A)
          })
    {
    }

    Pose(CMSISMat<3, 1> position, CMSISMat<3, 3> orientation)
    : position(position)
    {
        this->orientation = std::move(orientation);
    }

    inline float roll() const { return asinf(orientation.data[7]/cosf(pitch())); }

    inline float pitch() const { return asinf(-orientation.data[6]); }

    inline float yaw() const { return asinf(orientation.data[3]/cosf(pitch())); }

    Position<FRAME> position.
    CMSISMat<3, 3> orientation;
};  // struct Pose
}   // namespace tap::algorithms::transforms

#endif  // TAP_POSE_HPP_
