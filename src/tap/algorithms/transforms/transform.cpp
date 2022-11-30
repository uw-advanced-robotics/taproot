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
#include "transform.hpp"

namespace tap::algorithms::transforms
{
template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform(CMSISMat<3, 3>& rotation, CMSISMat<3, 1>& position)
    : rotation(rotation),
      position(position),
      tRotation()
{
    arm_mat_trans_f32(&this->rotation, &tRotation);
};

// template <typename SOURCE, typename TARGET>
// Transform<SOURCE, TARGET>::Transform(int& x, int& y, int& z, int& A, int& B, int& C){
//     // ???
// };

template <typename A, typename B, typename C>
Transform<A, C> Transform<A, C>::compose(Transform<A, B>& source, Transform<B, C>& target)
{
    // left multiply source transformation matrix with target transformation matrix to get
    // composition.
    CMSISMat<3, 3> newRot = source.rotation * target.rotation;
    CMSISMat<3, 1> newPos = source.position * target.position;
    return Transform<A, C>(&newRot, &newPos);
};

template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET> Transform<SOURCE, TARGET>::getInverse(Transform<SOURCE, TARGET>& tf)
{
    // negative transposed rotation matrix times original position = new position
    CMSISMat<3, 1> invPos = (-1 * tRotation) * position;
    return Transform<SOURCE, TARGET>(&tRotation, &invPos);
};

template <typename SOURCE, typename TARGET>
CMSISMat<3, 1> Transform<SOURCE, TARGET>::applyToPosition(CMSISMat<3, 1>& pos)
{
    CMSISMat<3, 1> newRot = rotation * (pos + position);
    return newRot;
};

void Transform<SOURCE, TARGET>::updateRotation(CMSISMat<3, 3>& newRot) {
    rotation = newRot;
}

void Transform<SOURCE, TARGET>::updatePosition(CMSISMat<3, 1>& newPos){
    position = newPos;
};

}  // namespace tap::algorithms::transforms
