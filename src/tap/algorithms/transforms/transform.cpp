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
{
    this->rotation = std::move(rotation);
    this->position = std::move(position);
    arm_mat_trans_f32(&this->rotation.matrix, &this->tRotation.matrix);
};

template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform(float x, float y, float z, float A, float B, float C)
{
    // For x-forward z-down (right-handed) coordinate system,
    // constructs rotation matrix where A, B, C = roll, pitch, yaw
    float data[9] = {
        std::cos(A) * std::cos(B),
        (std::cos(A) * std::sin(B) * std::sin(C)) - (std::sin(A) * std::cos(C)),
        (std::cos(A) * std::sin(B) * std::cos(C)) + std::sin(A) * std::sin(C),
        std::sin(A) * std::cos(B),
        std::sin(A) * std::sin(B) * std::sin(C) + std::cos(A) * std::cos(C),
        std::sin(A) * std::sin(B) * std::cos(C) - std::cos(A) * std::sin(C),
        -std::sin(B),
        std::cos(B) * std::sin(C),
        std::cos(B) * std::cos(C)};
    CMSISMat<3, 3> rot = CMSISMat<3, 3>(data);
    CMSISMat<3, 1> pos = CMSISMat<3, 1>({x, y, z});
    *this = Transform(rot, pos);
};

template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform()
{
    *this = Transform(0., 0., 0., 0., 0., 0.);
};

template <typename SRC, typename TARG, typename NEWTARGET>
Transform<SRC, NEWTARGET> compose(
    Transform<SRC, TARG>& transform1,
    Transform<TARG, NEWTARGET>& transform2)
{
    // left multiply transform1 transformation matrix with transform2 transformation matrix to get
    // composition.
    CMSISMat<3, 3> newRot = transform2.rotation * transform1.rotation;
    CMSISMat<3, 1> newPos = transform1.position + transform1.rotation * transform2.position;
    return Transform<SRC, NEWTARGET>(newRot, newPos);
};

template <typename SOURCE, typename TARGET>
Transform<TARGET, SOURCE> Transform<SOURCE, TARGET>::getInverse()
{
    // negative transposed rotation matrix times original position = new position
    CMSISMat<3, 1> invPos = tRotation * position;
    for (std::size_t i = 0; i < invPos.data.size(); i++)
    {
        invPos.data[i] = -invPos.data[i];
    }
    return Transform<TARGET, SOURCE>(tRotation, invPos);
};

template <typename SOURCE, typename TARGET>
CMSISMat<3, 1> Transform<SOURCE, TARGET>::applyToPosition(CMSISMat<3, 1>& pos)
{
    CMSISMat<3, 1> newPos = rotation * (pos + position);
    return newPos;
};

template <typename SOURCE, typename TARGET>
CMSISMat<3, 1> Transform<SOURCE, TARGET>::applyToVector(CMSISMat<3, 1>& vec)
{
    CMSISMat<3, 1> newVec = rotation * vec;
    return newVec;
};

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateRotation(CMSISMat<3, 3>& newRot)
{
    this->rotation = std::move(newRot);
    arm_mat_trans_f32(&this->rotation.matrix, &this->tRotation.matrix);
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateRotation(float A, float B, float C)
{
    float data[9] = {
        std::cos(A) * std::cos(B),
        (std::cos(A) * std::sin(B) * std::sin(C)) - (std::sin(A) * std::cos(C)),
        (std::cos(A) * std::sin(B) * std::cos(C)) + std::sin(A) * std::sin(C),
        std::sin(A) * std::cos(B),
        std::sin(A) * std::sin(B) * std::sin(C) + std::cos(A) * std::cos(C),
        std::sin(A) * std::sin(B) * std::cos(C) - std::cos(A) * std::sin(C),
        -std::sin(B),
        std::cos(B) * std::sin(C),
        std::cos(B) * std::cos(C)};
    this->rotation = CMSISMat<3, 3>(data);
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updatePosition(CMSISMat<3, 1>& newPos)
{
    this->position = std::move(newPos);
};
}  // namespace tap::algorithms::transforms
#endif
