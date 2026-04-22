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

#include "axis.hpp"
#include "vector.hpp"

namespace tap::algorithms::transforms
{
class Orientation
{
public:
    /**
     * Constructs an identity rotation
     */
    inline Orientation()
        : rotation({1, 0, 0, 0, 1, 0, 0, 0, 1}),
          rotationT({1, 0, 0, 0, 1, 0, 0, 0, 1}),
          rpy{0, 0, 0}
    {
    }

    inline Orientation(const float roll, const float pitch, const float yaw)
        : rotation(fromRollPitchYaw(roll, pitch, yaw)),
          rotationT(rotation.transpose()),
          rpy{roll, pitch, yaw}
    {
    }

    /* Costly; use rvalue reference whenever possible */
    inline Orientation(const CMSISMat<3, 3>& matrix) : rotation(matrix) { calculateRPY(); }
    inline Orientation(const CMSISMat<3, 3>& matrix, const CMSISMat<3, 3>& matrixT)
        : rotation(matrix),
          rotationT(matrixT)
    {
        calculateRPY();
    }

    inline Orientation(CMSISMat<3, 3>&& matrix) : rotation(std::move(matrix)) { calculateRPY(); }
    inline Orientation(CMSISMat<3, 3>&& matrix, CMSISMat<3, 3>&& matrixT)
        : rotation(std::move(matrix)),
          rotationT(std::move(matrixT))
    {
        calculateRPY();
    }

    Orientation& operator=(const Orientation& other)
    {
        this->rotation = other.rotation;
        this->rotationT = other.rotationT;
        this->rpy = other.rpy;
        return *this;
    }

    inline Orientation inverse() const { return Orientation(rotationT, rotation); }

    inline Orientation compose(const Orientation& other) const
    {
        return Orientation(this->rotation * other.rotation);
    }

    inline Vector apply(const Vector& vec) const
    {
        return Vector(this->rotation * vec.coordinates());
    }

    /**
     * Returns roll as values between [-pi, +pi].
     *
     * If pitch is completely vertical (-pi / 2 or pi / 2) then roll and yaw are gimbal-locked. In
     * this case, roll is taken to be 0.
     */
    inline float roll() const { return (*this)[Axis::ROLL]; }

    inline float pitch() const { return (*this)[Axis::PITCH]; }

    inline float yaw() const { return (*this)[Axis::YAW]; }

    const float& operator[](Axis a) const { return rpy[static_cast<int>(a)]; }

    const inline CMSISMat<3, 3>& matrix() const { return rotation; }

    modm::Quaternion<float> toQuaternion() const
    {
        float t;
        modm::Quaternion<float> q;
        if (rotation[2 * 3 + 2] < 0)
        {
            if (rotation[0 * 3 + 0] > rotation[1 * 3 + 1])
            {
                t = 1 + rotation[0 * 3 + 0] - rotation[1 * 3 + 1] - rotation[2 * 3 + 2];
                q = modm::Quaternion(
                    rotation[2 * 3 + 1] - rotation[1 * 3 + 2],
                    t,
                    rotation[0 * 3 + 1] + rotation[1 * 3 + 0],
                    rotation[2 * 3 + 0] + rotation[0 * 3 + 2]);
            }
            else
            {
                t = 1 - rotation[0 * 3 + 0] + rotation[1 * 3 + 1] - rotation[2 * 3 + 2];
                q = modm::Quaternion(
                    rotation[0 * 3 + 2] - rotation[2 * 3 + 0],
                    rotation[0 * 3 + 1] + rotation[1 * 3 + 0],
                    t,
                    rotation[1 * 3 + 2] + rotation[2 * 3 + 1]);
            }
        }
        else
        {
            if (rotation[0 * 3 + 0] < -rotation[1 * 3 + 1])
            {
                t = 1 - rotation[0 * 3 + 0] - rotation[1 * 3 + 1] + rotation[2 * 3 + 2];
                q = modm::Quaternion(
                    rotation[1 * 3 + 0] - rotation[0 * 3 + 1],
                    rotation[2 * 3 + 0] + rotation[0 * 3 + 2],
                    rotation[1 * 3 + 2] + rotation[2 * 3 + 1],
                    t);
            }
            else
            {
                t = 1 + rotation[0 * 3 + 0] + rotation[1 * 3 + 1] + rotation[2 * 3 + 2];
                q = modm::Quaternion(
                    t,
                    rotation[2 * 3 + 1] - rotation[1 * 3 + 2],
                    rotation[0 * 3 + 2] - rotation[2 * 3 + 0],
                    rotation[1 * 3 + 0] - rotation[0 * 3 + 1]);
            }
        }
        q *= 0.5 / sqrtf(t);
        return q;
    }

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

    /**
     * Constructs an `Orientation` from a unit quaternion.
     */
    static Orientation fromQuaternion(modm::Quaternion<float> q)
    {
        return fromQuaternion(q.w, q.x, q.y, q.z);
    }
    /**
     * Constructs an `Orientation` from a unit quaternion.
     */
    static Orientation fromQuaternion(float w, float x, float y, float z)
    {
        float xx = x * x;
        float xy = x * y;
        float xz = x * z;
        float xw = x * w;

        float yy = y * y;
        float yz = y * z;
        float yw = y * w;

        float zz = z * z;
        float zw = z * w;

        return Orientation(tap::algorithms::CMSISMat<3, 3>(
            {1 - 2 * (yy + zz),
             2 * (xy - zw),
             2 * (xz + yw),
             2 * (xy + zw),
             1 - 2 * (xx + zz),
             2 * (yz - xw),
             2 * (xz - yw),
             2 * (yz + xw),
             1 - 2 * (xx + yy)}));
    }

    friend class Transform;
    friend class DynamicOrientation;

protected:
    CMSISMat<3, 3> rotation, rotationT;
    std::array<float, 3> rpy;

    void calculateRPY()
    {
        float sin_p = -rotation.data[6];

        // Handle Gimbal Lock and float precision errors near 1.0 or -1.0
        // A threshold of 0.999999f catches anything within ~0.1 degrees of vertical
        if (sin_p >= 0.999999f)
        {
            // Pitch is +pi/2 (Straight down)
            rpy[static_cast<int>(Axis::Y)] = M_PI_2;
            rpy[static_cast<int>(Axis::X)] = 0.0f;

            // When roll is 0, m01 = -sin(yaw) and m11 = cos(yaw)
            rpy[static_cast<int>(Axis::Z)] = atan2(-rotation.data[1], rotation.data[4]);
        }
        else if (sin_p <= -0.999999f)
        {
            // Pitch is -pi/2
            rpy[static_cast<int>(Axis::Y)] = -M_PI_2;
            rpy[static_cast<int>(Axis::X)] = 0.0f;
            rpy[static_cast<int>(Axis::Z)] = atan2(-rotation.data[1], rotation.data[4]);
        }
        else
        {
            // Normal case
            rpy[static_cast<int>(Axis::Y)] = asinf(sin_p);
            rpy[static_cast<int>(Axis::X)] = atan2(rotation.data[7], rotation.data[8]);
            rpy[static_cast<int>(Axis::Z)] = atan2(rotation.data[3], rotation.data[0]);
        }
    }
};  // class Orientation
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_ORIENTATION_HPP_
