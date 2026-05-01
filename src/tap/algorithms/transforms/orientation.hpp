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
#include "tap/algorithms/wrapped_float.hpp"

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
        : matrix_({1, 0, 0, 0, 1, 0, 0, 0, 1}),
          matrixT_({1, 0, 0, 0, 1, 0, 0, 0, 1}),
          rpy{0, 0, 0}
    {
    }

    inline Orientation(const float roll, const float pitch, const float yaw)
        : matrix_(fromRollPitchYaw(roll, pitch, yaw)),
          matrixT_(matrix_.transpose())
    {
        calculateRPY();  // Don't use the input rpy since it may not be in the expected ranges
    }

    /* Costly; use rvalue reference whenever possible */
    inline Orientation(const CMSISMat<3, 3>& matrix)
        : matrix_(matrix),
          matrixT_(matrix_.transpose())
    {
        calculateRPY();
    }
    inline Orientation(const CMSISMat<3, 3>& matrix, const CMSISMat<3, 3>& matrixT)
        : matrix_(matrix),
          matrixT_(matrixT)
    {
        calculateRPY();
    }

    inline Orientation(CMSISMat<3, 3>&& matrix)
        : matrix_(std::move(matrix)),
          matrixT_(matrix_.transpose())
    {
        calculateRPY();
    }
    inline Orientation(CMSISMat<3, 3>&& matrix, CMSISMat<3, 3>&& matrixT)
        : matrix_(std::move(matrix)),
          matrixT_(std::move(matrixT))
    {
        calculateRPY();
    }

    /**
     * @brief A lightweight proxy to allow zero-overhead transposed multiplication.
     */
    struct TransposeProxy
    {
        const CMSISMat<3, 3>& rotationT;
    };

    inline Orientation compose(const Orientation& other) const { return *this * other; }

    inline Orientation operator*(const Orientation& other) const
    {
        return Orientation(this->matrix_ * other.matrix_);
    }

    inline Orientation operator*(const Orientation::TransposeProxy& other) const
    {
        return Orientation(this->matrix_ * other.rotationT);
    }

    inline Vector apply(const Vector& vec) const
    {
        return Vector(this->matrix_ * vec.coordinates());
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

    const inline CMSISMat<3, 3>& matrix() const { return matrix_; }

    const inline CMSISMat<3, 3>& matrixT() const { return matrixT_; }

    /**
     * @brief Returns a proxy object to use the cached transpose in mathematical operations
     * without constructing an intermediate Orientation object.
     * Example: Vector v2 = ori.T() * v1;
     * Example: Orientation o3 = ori1.T() * ori2;
     */
    inline TransposeProxy T() const { return {matrixT_}; }

    modm::Quaternion<float> toQuaternion() const
    {
        float t;
        modm::Quaternion<float> q;
        if (matrix_[2 * 3 + 2] < 0)
        {
            if (matrix_[0 * 3 + 0] > matrix_[1 * 3 + 1])
            {
                t = 1 + matrix_[0 * 3 + 0] - matrix_[1 * 3 + 1] - matrix_[2 * 3 + 2];
                q = modm::Quaternion(
                    matrix_[2 * 3 + 1] - matrix_[1 * 3 + 2],
                    t,
                    matrix_[0 * 3 + 1] + matrix_[1 * 3 + 0],
                    matrix_[2 * 3 + 0] + matrix_[0 * 3 + 2]);
            }
            else
            {
                t = 1 - matrix_[0 * 3 + 0] + matrix_[1 * 3 + 1] - matrix_[2 * 3 + 2];
                q = modm::Quaternion(
                    matrix_[0 * 3 + 2] - matrix_[2 * 3 + 0],
                    matrix_[0 * 3 + 1] + matrix_[1 * 3 + 0],
                    t,
                    matrix_[1 * 3 + 2] + matrix_[2 * 3 + 1]);
            }
        }
        else
        {
            if (matrix_[0 * 3 + 0] < -matrix_[1 * 3 + 1])
            {
                t = 1 - matrix_[0 * 3 + 0] - matrix_[1 * 3 + 1] + matrix_[2 * 3 + 2];
                q = modm::Quaternion(
                    matrix_[1 * 3 + 0] - matrix_[0 * 3 + 1],
                    matrix_[2 * 3 + 0] + matrix_[0 * 3 + 2],
                    matrix_[1 * 3 + 2] + matrix_[2 * 3 + 1],
                    t);
            }
            else
            {
                t = 1 + matrix_[0 * 3 + 0] + matrix_[1 * 3 + 1] + matrix_[2 * 3 + 2];
                q = modm::Quaternion(
                    t,
                    matrix_[2 * 3 + 1] - matrix_[1 * 3 + 2],
                    matrix_[0 * 3 + 2] - matrix_[2 * 3 + 0],
                    matrix_[1 * 3 + 0] - matrix_[0 * 3 + 1]);
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
        float pitch, yaw;
        vectorToSphericalCoords(dir, nullptr, &pitch, &yaw);
        return Orientation(0, pitch, yaw);
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

private:
    CMSISMat<3, 3> matrix_, matrixT_;
    std::array<float, 3> rpy;

    void calculateRPY()
    {
        float sin_p = -matrix_.data[6];

        // Handle Gimbal Lock and float precision errors near 1.0 or -1.0
        // A threshold of 0.999999f catches anything within ~0.1 degrees of vertical
        if (sin_p >= 0.999999f)
        {
            // Pitch is +pi/2 (Straight down)
            rpy[static_cast<int>(Axis::Y)] = M_PI_2;
            rpy[static_cast<int>(Axis::X)] = 0.0f;

            // When roll is 0, m01 = -sin(yaw) and m11 = cos(yaw)
            rpy[static_cast<int>(Axis::Z)] = atan2(-matrix_.data[1], matrix_.data[4]);
        }
        else if (sin_p <= -0.999999f)
        {
            // Pitch is -pi/2
            rpy[static_cast<int>(Axis::Y)] = -M_PI_2;
            rpy[static_cast<int>(Axis::X)] = 0.0f;
            rpy[static_cast<int>(Axis::Z)] = atan2(-matrix_.data[1], matrix_.data[4]);
        }
        else
        {
            // Normal case
            rpy[static_cast<int>(Axis::Y)] = asinf(sin_p);
            rpy[static_cast<int>(Axis::X)] = atan2(matrix_.data[7], matrix_.data[8]);
            rpy[static_cast<int>(Axis::Z)] = atan2(matrix_.data[3], matrix_.data[0]);
        }
    }
};  // class Orientation

/**
 * @brief Multiplies a 3x3 rotation matrix by a 3D vector.
 */
inline Vector operator*(const Orientation& a, const Vector& b)
{
    return Vector(a.matrix() * b.coordinates());
}

/**
 * @brief Multiplies a transposed rotation matrix by a 3D vector.
 */
inline Vector operator*(const Orientation::TransposeProxy& a, const Vector& b)
{
    return Vector(a.rotationT * b.coordinates());
}

/**
 * @brief Composes a transposed orientation with another orientation.
 */
inline Orientation operator*(const Orientation::TransposeProxy& a, const Orientation& b)
{
    return Orientation(a.rotationT * b.matrix());
}
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_ORIENTATION_HPP_
