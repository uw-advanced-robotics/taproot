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
#ifndef TAPROOT_TRANSFORM_CPP_
#define TAPROOT_TRANSFORM_CPP_

#include "transform.hpp"
namespace tap::algorithms::transforms
{
template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform(CMSISMat<3, 3>& rotation, CMSISMat<3, 1>& position)
    : rotation(rotation),
      position(position),
      tRotation()
{
    arm_mat_trans_f32(&this->rotation.matrix, &this->tRotation.matrix);
};

template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform(int& x, int& y, int& z, int& A, int& B, int& C)
{
    // For x forward z down coordinate system,
    // constructs rotation matrix where C, B, A = yaw, pitch, roll
    int data[3][3] = {
        {std::cos(C) * std::cos(B),
         (std::cos(C) * std::sin(B) * std::sin(A)) - (std::sin(C) * std::cos(A)),
         (std::cos(C) * std::sin(B) * std::cos(A)) + std::sin(C) * std::sin(A)},
        {std::sin(C) * std::cos(B),
         std::sin(C) * std::sin(B) * std::sin(A) + std::cos(C) * std::cos(A),
         std::sin(C) * std::sin(B) * std::cos(A) - std::cos(C) * std::sin(A)},
        {-std::sin(B), std::cos(B) * std::sin(A), std::cos(B) * std::cos(A)}};
    CMSISMat<3, 3> rot = CMSISMat<3, 3>(data);
    CMSISMat<3, 1> pos = CMSISMat<3, 1>({x, y, z});
    Transform(&rot, &pos);
};

template <typename SOURCE, typename TARGET, typename NEWTARGET>
Transform<SOURCE, NEWTARGET> compose(
    Transform<SOURCE, TARGET>& source,
    Transform<TARGET, NEWTARGET>& target)
{
    // left multiply source transformation matrix with target transformation matrix to get
    // composition.
    CMSISMat<3, 3> newRot = source.rotation.matrix * target.rotation.matrix;
    CMSISMat<3, 1> newPos = source.position.matrix * target.position.matrix;
    return Transform<SOURCE, NEWTARGET>(&newRot, &newPos);
};

template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET> Transform<SOURCE, TARGET>::getInverse(Transform<SOURCE, TARGET>& tf)
{
    // negative transposed rotation matrix times original position = new position
    CMSISMat<3, 1> invPos = tRotation * position;
    for (int i = 0; i < invPos.data.size(); i++)
    {
        invPos.data[i] = -invPos.data[i];
    }
    return Transform<SOURCE, TARGET>(&tRotation, &invPos);
};

template <typename SOURCE, typename TARGET>
CMSISMat<3, 1> Transform<SOURCE, TARGET>::applyToPosition(CMSISMat<3, 1>& pos)
{
    CMSISMat<3, 1> newRot = rotation * (pos + position);
    return newRot;
};

template <typename SOURCE, typename TARGET>
CMSISMat<3, 1> Transform<SOURCE, TARGET>::applyToVector(CMSISMat<3, 1>& pos)
{
    CMSISMat<3, 1> newRot = (rotation * pos) + position;
    return newRot;
};

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateRotation(CMSISMat<3, 3>& newRot)
{
    this->rotation = newRot;
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updatePosition(CMSISMat<3, 1>& newPos)
{
    this->position = newPos;
};
}  // namespace tap::algorithms::transforms
#endif
