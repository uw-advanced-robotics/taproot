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

#ifndef TAPROOT_EULER_ANGLES_HPP_
#define TAPROOT_EULER_ANGLES_HPP_

#include "cmsis_mat.hpp"

namespace tap::algorithms
{
CMSISMat<3, 3> fromEulerAngles(const float roll, const float pitch, const float yaw)
{
    return CMSISMat<3, 3>(
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

}  // namespace tap::algorithms

#endif  // TAPROOT_EULER_ANGLES_HPP_