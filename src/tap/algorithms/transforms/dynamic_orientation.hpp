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
class DynamicOrientation : public Orientation
{
public:
    inline DynamicOrientation() : Orientation(), angularVelocity() {}

    inline DynamicOrientation(
        const float roll,
        const float pitch,
        const float yaw,
        const float rollVel,
        const float pitchVel,
        const float yawVel)
        : Orientation(roll, pitch, yaw),
          angularVelocity(AngularVelocity::skewMatFromAngVel(rollVel, pitchVel, yawVel))
    {
    }

    inline DynamicOrientation(
        const CMSISMat<3, 3>&& orientation,
        const CMSISMat<3, 3>&& angularVelocity)
        : Orientation(orientation),
          angularVelocity(std::move(angularVelocity))
    {
    }

    /* Costly; use rvalue reference whenever possible */
    inline DynamicOrientation(
        const CMSISMat<3, 3>& orientation,
        const CMSISMat<3, 3>& angularVelocity)
        : Orientation(orientation),
          angularVelocity(angularVelocity)
    {
    }

    inline DynamicOrientation(Orientation&& orientation, AngularVelocity&& angularVelocity)
        : Orientation(std::move(orientation.rotation)),
          angularVelocity(std::move(angularVelocity.matrix_))
    {
    }

    inline DynamicOrientation(Orientation& orientation, AngularVelocity& angularVelocity)
        : Orientation(orientation.rotation),
          angularVelocity(angularVelocity.matrix_)
    {
    }

    DynamicOrientation compose(const DynamicOrientation& other) const
    {
        return DynamicOrientation(
            this->rotation * other.rotation,
            this->angularVelocity + this->rotation * other.angularVelocity * this->rotationT);
    }

    DynamicOrientation inverse() const
    {
        return DynamicOrientation(
            this->rotationT,
            -(this->rotationT * this->angularVelocity * this->rotation));
    }

    inline AngularVelocity getAngularVelocity() const { return AngularVelocity(angularVelocity); }

    /**
     * @brief Get the roll velocity
     */
    inline float getRollVelocity() const { return angularVelocity.data[0 * 3 + 2]; }

    /**
     * @brief Get the pitch velocity
     */
    inline float getPitchVelocity() const { return -angularVelocity.data[1 * 3 + 2]; }

    /**
     * @brief Get the yaw velocity
     */
    inline float getYawVelocity() const { return -angularVelocity.data[0 * 3 + 1]; }

    friend class Transform;

private:
    CMSISMat<3, 3> angularVelocity;

};  // class DynamicOrientation
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_DYNAMIC_ORIENTATION_HPP_
