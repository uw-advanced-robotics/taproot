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

#include "orientation.hpp"
#include "position.hpp"
#include "frame.hpp"

namespace tap::algorithms::transforms
{

template <Frame FRAME>
class Pose
{
public:
    inline Pose(const float x, const float y, const float z, const float roll, const float pitch, const float yaw)
        : position_(x, y, z),
          orientation_(roll, pitch, yaw)
    {
    }

    inline Pose(CMSISMat<3, 1> position, CMSISMat<3, 3> orientation)
        : position_(std::move(position)),
          orientation_(std::move(orientation))
    {
    }

    const inline Position<FRAME>& position() const { return position_; }
    const inline Orientation<FRAME>& orientation() const { return orientation_; }

private:
    Position<FRAME> position_;
    Orientation<FRAME> orientation_;
};  // struct Pose
}  // namespace tap::algorithms::transforms

#endif  // TAP_POSE_HPP_
