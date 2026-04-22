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
    inline DynamicOrientation() : rotation(), angularVelocity() {}

    inline DynamicOrientation(
        const float roll,
        const float pitch,
        const float yaw,
        const float rollVel,
        const float pitchVel,
        const float yawVel)
        : rotation(roll, pitch, yaw),
          angularVelocity(rollVel, pitchVel, yawVel)
    {
    }

    inline DynamicOrientation(
        const CMSISMat<3, 3>&& orientation,
        const CMSISMat<3, 3>&& angularVelocity)
        : rotation(orientation),
          angularVelocity(angularVelocity)
    {
    }

    /* Costly; use rvalue reference whenever possible */
    inline DynamicOrientation(
        const CMSISMat<3, 3>& orientation,
        const CMSISMat<3, 3>& angularVelocity)
        : rotation(orientation),
          angularVelocity(angularVelocity)
    {
    }

    inline DynamicOrientation(Orientation&& orientation, AngularVelocity&& angularVelocity)
        : rotation(orientation),
          angularVelocity(angularVelocity)
    {
    }

    inline DynamicOrientation(Orientation& orientation, AngularVelocity& angularVelocity)
        : rotation(orientation),
          angularVelocity(angularVelocity)
    {
    }

    DynamicOrientation compose(const DynamicOrientation& other) const
    {
        return DynamicOrientation(
            this->rotation.rotation * other.rotation.rotation,
            this->angularVelocity.toSkewMatrix() + this->rotation.rotation *
                                                       other.angularVelocity.toSkewMatrix() *
                                                       this->rotation.rotationT);
    }

    DynamicOrientation inverse() const
    {
        return DynamicOrientation(
            this->rotation.rotationT,
            -(this->rotation.rotationT * this->angularVelocity.toSkewMatrix() *
              this->rotation.rotation));
    }

    inline const Orientation& getRotation() const { return rotation; }
    inline const AngularVelocity& getAngularVelocity() const
    {
        return AngularVelocity(angularVelocity);
    }

    inline float getRoll() const { return rotation.roll(); }
    inline float getPitch() const { return rotation.pitch(); }
    inline float getYaw() const { return rotation.yaw(); }

    inline float getRollVelocity() const { return angularVelocity.getRollVelocity(); }
    inline float getPitchVelocity() const { return angularVelocity.getPitchVelocity(); }
    inline float getYawVelocity() const { return angularVelocity.getYawVelocity(); }

    friend class Transform;

private:
    const Orientation rotation;
    const AngularVelocity angularVelocity;

};  // class DynamicOrientation
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_DYNAMIC_ORIENTATION_HPP_
