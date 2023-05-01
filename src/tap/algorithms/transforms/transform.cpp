#ifndef TAPROOT_TRANSFORM_CPP_
#define TAPROOT_TRANSFORM_CPP_

#include "transform.hpp"
namespace tap::algorithms::transforms
{
template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform(CMSISMat<3, 1>& translation, CMSISMat<3, 3>& rotation)
{
    this->translation = std::move(translation);
    this->rotation = std::move(rotation);
    arm_mat_trans_f32(&this->rotation.matrix, &this->tRotation.matrix);
}

template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform(float x, float y, float z, float A, float B, float C)
{
    CMSISMat<3, 1> translation = CMSISMat<3, 1>({x, y, z});
    float data[9] = {
        std::cos(C) * std::cos(B),
        (std::cos(C) * std::sin(B) * std::sin(A)) - (std::sin(C) * std::cos(A)),
        (std::cos(C) * std::sin(B) * std::cos(A)) + std::sin(C) * std::sin(A),
        std::sin(C) * std::cos(B),
        std::sin(C) * std::sin(B) * std::sin(A) + std::cos(C) * std::cos(A),
        std::sin(C) * std::sin(B) * std::cos(A) - std::cos(C) * std::sin(A),
        -std::sin(B),
        std::cos(B) * std::sin(A),
        std::cos(B) * std::cos(A)};
    CMSISMat<3, 3> rotation = CMSISMat<3, 3>(data);
    *this = Transform(translation, rotation);
}

template <typename SOURCE, typename TARGET>
Transform<SOURCE, TARGET>::Transform()
{
    *this = Transform(0., 0., 0., 0., 0., 0.);
}

template <typename SOURCE, typename TARGET>
Transform<TARGET, SOURCE> Transform<SOURCE, TARGET>::getInverse() const
{
    CMSISMat<3, 1> invTranslation = tRotation * translation;
    invTranslation = -invTranslation;
    return Transform<TARGET, SOURCE>(invTranslation, tRotation);
}

template <typename SOURCE, typename TARGET>
CMSISMat<3, 1> Transform<SOURCE, TARGET>::applyToPosition(const CMSISMat<3, 1>& pos) const
{
    CMSISMat<3, 1> newPos = tRotation * (pos - translation);
    return newPos;
}

template <typename SOURCE, typename TARGET>
CMSISMat<3, 1> Transform<SOURCE, TARGET>::applyToVector(const CMSISMat<3, 1>& vec) const
{
    CMSISMat<3, 1> newVec = tRotation * vec;
    return newVec;
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateRotation(CMSISMat<3, 3>& newRotation)
{
    this->rotation = std::move(newRotation);
    arm_mat_trans_f32(&this->rotation.matrix, &this->tRotation.matrix);
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateRotation(float A, float B, float C)
{
    float data[9] = {
        std::cos(C) * std::cos(B),
        (std::cos(C) * std::sin(B) * std::sin(A)) - (std::sin(C) * std::cos(A)),
        (std::cos(C) * std::sin(B) * std::cos(A)) + std::sin(C) * std::sin(A),
        std::sin(C) * std::cos(B),
        std::sin(C) * std::sin(B) * std::sin(A) + std::cos(C) * std::cos(A),
        std::sin(C) * std::sin(B) * std::cos(A) - std::cos(C) * std::sin(A),
        -std::sin(B),
        std::cos(B) * std::sin(A),
        std::cos(B) * std::cos(A)};
    CMSISMat<3, 3> newRotation = CMSISMat<3, 3>(data);
    updateRotation(newRotation);
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateTranslation(const CMSISMat<3, 1>& newTranslation)
{
    this->translation = std::move(newTranslation);
}

template <typename SOURCE, typename TARGET>
void Transform<SOURCE, TARGET>::updateTranslation(float x, float y, float z)
{
    CMSISMat<3, 1> newTranslation = CMSISMat<3, 1>({x, y, z});
    this->translation = std::move(newTranslation);
}

template <typename A, typename B, typename C>
Transform<A, C> compose(const Transform<A, B>& first, const Transform<B, C>& second)
{
    CMSISMat<3, 3> newRotation = first.rotation * second.rotation;
    CMSISMat<3, 1> newTranslation = first.translation + first.rotation * second.translation;
    return Transform<A, C>(newTranslation, newRotation);
}
}  // namespace tap::algorithms::transforms
#endif
