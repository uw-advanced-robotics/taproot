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

#ifndef TAPROOT_POSITION_HPP_
#define TAPROOT_POSITION_HPP_

#include "tap/algorithms/cmsis_mat.hpp"

namespace tap::algorithms::transforms
{

class Position
{
public:
    /* Constructors */
    Position(float x, float y, float z) : coordinates_({x, y, z}) {}

    /* Use rvalue reference */
    Position(const Position&& other) : coordinates_(std::move(other.coordinates_)) {}

    Position(const Position& other) : coordinates_(CMSISMat(other.coordinates_)) {}

    Position(const CMSISMat<3, 1>& coordinates) : coordinates_(coordinates) {}

    Position(CMSISMat<3, 1>&& coordinates) : coordinates_(std::move(coordinates)) {}

    /* Getters */

    float x() const { return coordinates_.data[0]; }

    float y() const { return coordinates_.data[1]; }

    float z() const { return coordinates_.data[2]; }

    /* Operators */
    Position operator-(const Position& other) const
    {
        return Position(this->coordinates_ - other.coordinates_);
    }

    Position operator+(const Position& vector) const
    {
        return Position(this->coordinates_ + vector.coordinates_);
    }

    inline CMSISMat<3, 1> coordinates() const { return this->coordinates_; }

    friend class Transform;

private:
    CMSISMat<3, 1> coordinates_;
};  // class Position
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_POSITION_HPP_
