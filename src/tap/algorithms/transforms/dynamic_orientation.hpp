/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef TAPROOT_DYNAMIC_ORIENTATION_HPP_
#define TAPROOT_DYNAMIC_ORIENTATION_HPP_

#include "tap/algorithms/cmsis_mat.hpp"

#include "angular_velocity.hpp"
#include "orientation.hpp"

namespace tap::algorithms::transforms
{
class DynamicOrientation
{
public:
    inline DynamicOrientation(
        const float roll,
        const float pitch,
        const float yaw,
        const float rollVel,
        const float pitchVel,
        const float yawVel)
        : orientation(Orientation::fromEulerAngles(roll, pitch, yaw)),
          angularVelocity(AngularVelocity::skewMatFromAngVel(rollVel, pitchVel, yawVel))
    {
    }

    inline DynamicOrientation(Orientation&& orientation, AngularVelocity&& angularVelocity)
        : orientation(std::move(orientation.matrix_)),
          angularVelocity(std::move(angularVelocity.matrix_))
    {
    }

    inline DynamicOrientation(Orientation& orientation, AngularVelocity& angularVelocity)
        : orientation(orientation.matrix_),
          angularVelocity(angularVelocity.matrix_)
    {
    }

    /* rvalue reference */
    inline DynamicOrientation(DynamicOrientation&& other)
        : orientation(std::move(other.orientation)),
          angularVelocity(std::move(other.angularVelocity))
    {
    }

    /* Costly; use rvalue reference whenever possible */
    inline DynamicOrientation(DynamicOrientation& other)
        : orientation(other.orientation),
          angularVelocity(other.angularVelocity)
    {
    }

    inline DynamicOrientation(
        const CMSISMat<3, 3>&& orientation,
        const CMSISMat<3, 3>&& angularVelocity)
        : orientation(std::move(orientation)),
          angularVelocity(std::move(angularVelocity))
    {
    }

    /* Costly; use rvalue reference whenever possible */
    inline DynamicOrientation(
        const CMSISMat<3, 3>& orientation,
        const CMSISMat<3, 3>& angularVelocity)
        : orientation(orientation),
          angularVelocity(angularVelocity)
    {
    }

    friend class Transform;

private:
    CMSISMat<3, 3> orientation;

    CMSISMat<3, 3> angularVelocity;

};  // class DynamicOrientation
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_DYNAMIC_ORIENTATION_HPP_
