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
    inline DynamicOrientation(const float rollVel, const float pitchVel, const float yawVel)
        : matrix_((rollVel, pitchVel, yawVel))
    {
    }

    inline DynamicOrientation(Orientation&& orientation, AngularVelocity&& angularVelocity)
        : orientation(orientation),
          angularVelocity(angularVelocity)
    {
    }

    inline DynamicOrientation(Orientation& orientation, AngularVelocity& angularVelocity)
        : orientation(orientation),
          angularVelocity(angularVelocity)
    {
    }

    /* rvalue reference */
    inline DynamicOrientation(DynamicOrientation&& other) : matrix_(std::move(other.matrix_)) {}

    /* Costly; use rvalue reference whenever possible */
    inline DynamicOrientation(DynamicOrientation& other) : matrix_(CMSISMat(other.matrix_)) {}

    /* Costly; use rvalue reference whenever possible */
    inline DynamicOrientation(const CMSISMat<3, 3>& matrix) : matrix_(matrix) {}

    inline DynamicOrientation(CMSISMat<3, 3>&& matrix) : matrix_(std::move(matrix)) {}

private:
    Orientation orientation;

    AngularVelocity angularVelocity;

};  // class DynamicOrientation
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_DYNAMIC_ORIENTATION_HPP_
