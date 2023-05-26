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

namespace tap::algorithms::transforms
{

// Forward declaration for vector.hpp
template <typename FRAME>
class Vector;

template <typename FRAME>
class Position
{
public:
    Position(float x, float y, float z)
    : coordinates({x, y, z}) {}

    Position(CMSISMat<3,1>& coordinates)
    {
        this->coordinates = std::move(coordinates);
    }

    inline float x() const { return coordinates.data[0]; }

    inline float y() const { return coordinates.data[1]; }

    inline float z() const { return coordinates.data[2]; }

    inline Vector<FRAME> operator-(Position<FRAME>& other) const { return Vector<FRAME>(this->coordinates - other.coordinates); }

    inline Position<FRAME> operator+(const Vector<FRAME>& vector) const { return Position<FRAME>(this->coordinates + vector.coordinates); }

    const inline CMSISMat<3, 1>& coordinates() const { return coordinates; }

private:
    CMSISMat<3, 1> coordinates;
};  // class Position
}   // namespace tap::algorithms::transforms

#endif  // TAPROOT_POSITION_HPP_
