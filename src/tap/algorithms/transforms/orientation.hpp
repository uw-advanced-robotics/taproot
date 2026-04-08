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

#include "vector.hpp"

namespace tap::algorithms::transforms
{
class Orientation
{
public:
    /**
     * Constructs an identity rotation
     */
    inline Orientation() : rotation({1, 0, 0, 0, 1, 0, 0, 0, 1}), roll_(0), pitch_(0), yaw_(0) {}

    inline Orientation(const float roll, const float pitch, const float yaw)
        : rotation(fromRollPitchYaw(roll, pitch, yaw)),
          roll_(roll),
          pitch_(pitch),
          yaw_(yaw)
    {
    }

    /* rvalue reference */
    inline Orientation(Orientation&& other)
        : rotation(std::move(other.rotation)),
          roll_(other.roll_),
          pitch_(other.pitch_),
          yaw_(other.yaw_)
    {
    }

    /* Costly; use rvalue reference whenever possible */
    inline Orientation(Orientation& other)
        : rotation(CMSISMat(other.rotation)),
          roll_(other.roll_),
          pitch_(other.pitch_),
          yaw_(other.yaw_)
    {
    }

    /* Costly; use rvalue reference whenever possible */
    inline Orientation(const CMSISMat<3, 3>& matrix) : rotation(matrix) { calculateRPY(); }

    inline Orientation(CMSISMat<3, 3>&& matrix) : rotation(std::move(matrix)) { calculateRPY(); }

    inline Orientation compose(const Orientation& other) const
    {
        return Orientation(this->rotation * other.rotation);
    }

    /**
     * Returns roll as values between [-pi, +pi].
     *
     * If pitch is completely vertical (-pi / 2 or pi / 2) then roll and yaw are gimbal-locked. In
     * this case, roll is taken to be 0.
     */
    inline float roll() const { return roll_; }

    inline float pitch() const { return pitch_; }

    inline float yaw() const { return yaw_; }

    const inline CMSISMat<3, 3>& matrix() const { return rotation; }

    /**
     * Generates a 3x3 rotation matrix from roll pitch yaw (in radians)
     */
    static CMSISMat<3, 3> fromRollPitchYaw(const float roll, const float pitch, const float yaw)
    {
        return tap::algorithms::CMSISMat<3, 3>(
            {cosf(yaw) * cosf(pitch),
             (cosf(yaw) * sinf(pitch) * sinf(roll)) - (sinf(yaw) * cosf(roll)),
             (cosf(yaw) * sinf(pitch) * cosf(roll)) + sinf(yaw) * sinf(roll),
             sinf(yaw) * cosf(pitch),
             sinf(yaw) * sinf(pitch) * sinf(roll) + cosf(yaw) * cosf(roll),
             sinf(yaw) * sinf(pitch) * cosf(roll) - cosf(yaw) * sinf(roll),
             -sinf(pitch),
             cosf(pitch) * sinf(roll),
             cosf(pitch) * cosf(roll)});
    }

    /**
     * Constructs an `Orientation` from a direction vector. Magnitude is ignored, and roll is always
     * 0.
     */
    static Orientation fromDirectionVector(Vector dir)
    {
        float mag = dir.magnitude();
        Vector planar(dir.x(), dir.y(), 0);
        return Orientation(0, asinf(planar.magnitude() / mag), atan2f(dir.y(), dir.x()));
    }

    friend class Transform;
    friend class DynamicOrientation;

protected:
    CMSISMat<3, 3> rotation;
    float roll_, pitch_, yaw_;

    void calculateRPY()
    {
        roll_ = atan2(rotation.data[7], rotation.data[8]);
        pitch_ = asinf(-rotation.data[6]);
        yaw_ = atan2(rotation.data[3], rotation.data[0]);
    }
};  // class Orientation
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_ORIENTATION_HPP_
