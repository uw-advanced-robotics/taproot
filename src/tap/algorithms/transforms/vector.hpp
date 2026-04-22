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

#ifndef TAPROOT_VECTOR_HPP_
#define TAPROOT_VECTOR_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"

#include "axis.hpp"

namespace tap::algorithms::transforms
{
// forward declare position to avoid circular dependency
class Position;

class Vector
{
public:
    Vector() : coordinates_({0, 0, 0}) {}

    Vector(float x, float y, float z) : coordinates_({x, y, z}) {}

    Vector(Vector&& other) : coordinates_(std::move(other.coordinates_)) {}

    Vector(const Vector& other) : coordinates_(CMSISMat(other.coordinates_)) {}

    /**
     * Costly copy constructor
     */
    Vector(const CMSISMat<3, 1>& coordinates) : coordinates_(CMSISMat(coordinates)) {}

    Vector(CMSISMat<3, 1>&& coordinates) : coordinates_(std::move(coordinates)) {}

    template <Axis A, bool NEG = false>
    inline static Vector axis()
    {
        Vector v;
        v[A] = NEG ? -1 : 1;
        return v;
    }

    inline float x() const { return (*this)[Axis::X]; }
    inline float y() const { return (*this)[Axis::Y]; }
    inline float z() const { return (*this)[Axis::Z]; }

    const float& operator[](Axis a) const { return coordinates_[static_cast<int>(a)]; }
    const float& operator[](int i) const { return coordinates_[i]; }

    inline Vector& operator=(const Vector& other)
    {
        this->coordinates_ = other.coordinates_;
        return *this;
    }

    // inline Vector operator+(const Position& other) const
    // {
    //     return Vector(this->coordinates_ + other.coordinates());
    // }

    inline Vector operator+(const Vector& other) const
    {
        return Vector(this->coordinates_ + other.coordinates_);
    }

    inline Vector operator-(const Vector& other) const
    {
        return Vector(this->coordinates_ - other.coordinates_);
    }

    inline Vector operator*(const float scale) const { return Vector(this->coordinates_ * scale); }

    inline Vector operator/(const float scale) const { return Vector(this->coordinates_ / scale); }

    inline static float dot(const Vector& a, const Vector& b)
    {
        return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    }

    inline float dot(const Vector& other) const { return dot(*this, other); }

    inline static Vector cross(const Vector& a, const Vector& b)
    {
        return Vector(tap::algorithms::cross(a.coordinates(), b.coordinates()));
    }

    inline Vector cross(const Vector& other) const { return cross(*this, other); }

    const inline CMSISMat<3, 1>& coordinates() const { return coordinates_; }

    inline float magnitude() const { return sqrt(dot(*this, *this)); }

    inline Vector normalize() const { return (*this) / this->magnitude(); }

    Vector project(const Vector& onto) { return onto.normalize() * this->dot(onto); }

    friend class Transform;
    friend class DynamicPosition;

private:
    CMSISMat<3, 1> coordinates_;
};  // class Vector
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_VECTOR_HPP_
