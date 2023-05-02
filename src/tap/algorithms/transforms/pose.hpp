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

#ifndef TAPROOT_POSE_HPP_
#define TAPROOT_POSE_HPP_

#include "transform.hpp"

namespace tap::algorithms::transforms
{

template <typename FRAME>
class Pose
{
    Pose(float x, float y, float z, float A, float B, float C)
    : coords({x, y, z}), rotation(rotationMatrix(A, B, C)) {}

    Pose(CMSISMat<3, 1> position, CMSISMat<3, 3> orientation)
    : coords(position), rotation(orientation) {}

    inline void updateCoordinates(float x, float y, float z)
    {
        coords.data = {x, y, z};
    }

    inline Pose<NEW_FRAME> transform(Transform<FRAME, NEW_FRAME> tf)
    {
        return Pose<NEW_FRAME>(coords + tf.getTranslation(), rotation*tf.getRotation());
    }

    inline CMSISMat<3, 1> getCoordinates() { return coords; }

    inline CMSISMat<3, 3> getOrientation() { return rotation; }

    inline float getX() { return coords.data[0]; }

    inline float getY() { return coords.data[1]; }

    inline float getZ() { return coords.data[2]; }

    inline float getA() { return asinf(rotation.data[7]/cosf(getB())); }

    inline float getB() { return asinf(-rotation.data[6]); }

    inline float getC() { return asinf(rotation.data[3]/cosf(getB())); }

private:
    CMSISMat<3, 1> coords;
    CMSISMat<3, 3> rotation;
};
}   // namespace tap::algorithms::transforms

#endif  // TAP_POSE_HPP_
