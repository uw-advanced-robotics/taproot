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
{
    this->rotation = std::move(rotation);
    this->position = std::move(position);
    arm_mat_trans_f32(&this->rotation.matrix, &this->tRotation.matrix);
}

template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform(float x, float y, float z, float A, float B, float C)
{
    CMSISMat<3, 3> rot = rotationMatrix(A, B, C);
    CMSISMat<3, 1> pos = CMSISMat<3, 1>({x, y, z});
    *this = Transform(rot, pos);
}

template <typename SOURCE, typename TARGET>
Transform<TARGET, SOURCE> Transform<SOURCE, TARGET>::getInverse() const
{
    // negative transposed rotation matrix times original position = new position
    CMSISMat<3, 1> invTranslation = tRotation * translation;
    invTranslation = -invTranslation;
    return Transform<TARGET, SOURCE>(tRotation, invTranslation);
}

template <typename SOURCE, typename TARGET>
CMSISMat<3, 1> Transform<SOURCE, TARGET>::applyToVector(const CMSISMat<3, 1>& vec) const
{
    CMSISMat<3, 1> newVec = tRotation * vec;
    return newVec;
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateRotation(CMSISMat<3, 3>& newRot)
{
    this->rotation = std::move(newRot);
    arm_mat_trans_f32(&this->rotation.matrix, &this->tRotation.matrix);
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateTranslation(float x, float y, float z)
{
    CMSISMat<3, 1>& newTranslation({x, y, z});
    this->position = std::move(newTranslation);
}

template <typename A, typename B, typename C>
Transform<A, C> compose(const Transform<A, B>& first, const Transform<B, C>& second)
{
    CMSISMat<3, 3> newRot = first.rotation * second.rotation;
    CMSISMat<3, 1> newPos = first.translation + first.rotation * second.translation;
    return Transform<A, C>(newRot, newPos);
}
}  // namespace tap::algorithms::transforms
