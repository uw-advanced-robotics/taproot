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
#include "tap/algorithms/cmsis_mat.hpp"
namespace tap::algorithms::transforms
{
template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform(CMSISMat<3, 3>& rotation, CMSISMat<3, 1>& translation)
{
    this->rotation = std::move(rotation);
    this->translation = std::move(position);
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
Position<TARGET> apply(Position<SOURCE>& position)
{
    return Position<TARGET>(position.coordinates + translation);
}

template <typename SOURCE, typename TARGET>
Pose<TARGET> apply(Pose<SOURCE>& pose)
{
    return Pose<TARGET>(pose.position.coordinates + translation, pose.orientation*rotation);
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
Vector<TARGET> Transform<SOURCE, TARGET>::apply(Vector<SOURCE>& vector)
{
    return Vector<TARGET>(rotation * vector.coordinates);
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateRotation(CMSISMat<3, 3>& newRotation)
{
    this->rotation = std::move(newRotation);
    this->tRotation = rotation.transpose();
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateTranslation(float x, float y, float z)
{
    CMSISMat<3, 1>& newTranslation({x, y, z});
    this->translation = std::move(newTranslation);
}

template <typename A, typename B, typename C>
Transform<A, C> compose(const Transform<A, B>& first, const Transform<B, C>& second)
{
    CMSISMat<3, 3> newRot = first.rotation * second.rotation;
    CMSISMat<3, 1> newPos = first.translation + first.rotation * second.translation;
    return Transform<A, C>(newRot, newPos);
}
}  // namespace tap::algorithms::transforms
