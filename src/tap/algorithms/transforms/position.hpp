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

#include "transform.hpp"

namespace tap::algorithms::transforms
{

template <typename FRAME>
class Position
{
    Position(float x, float y, float z)
    : coords({x, y, z}) {}

    Position(CMSISMat<3,1>& coordinates)
    : coords(coordinates) {}

    Position<NEW_FRAME> transform(Transform<FRAME, NEW_FRAME> tf)
    {
        return Position<NEW_FRAME>(coords + tf.getTranslation());
    }

    inline void updateCoordinates(float x, float y, float z)
    {
        coords.data = {x, y, z};
    }

    inline CMSISMat<3, 1> getCoordinates() { return coords; };

    inline float getX() { return coords.data[0]; }

    inline float getY() { return coords.data[1]; }

    inline float getZ() { return coords.data[2]; }

private:
    CMSISMat<3, 1> coords;
};
}   // namespace tap::algorithms::transforms

#endif  // TAPROOT_POSITION_HPP_
