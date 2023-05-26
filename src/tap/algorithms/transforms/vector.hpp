
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

namespace tap::algorithms::transforms
{
template<typename FRAME>
struct Vector
{
    Vector(float x, float y, float z)
    : entries({x, y, z})

    Vector(CMSISMat<3,1>& vector)
    {
        this->entries = std::move(vector);
    }

    inline float x() const { return entries.data[0]; }

    inline float y() const { return entries.data[1]; }

    inline float z() const { return entries.data[2]; }

    CMSISMat<3, 1> entries;
};  // struct Vector
}   // namespace tap::algorithms::transforms

#endif  // TAPROOT_VECTOR_HPP_
