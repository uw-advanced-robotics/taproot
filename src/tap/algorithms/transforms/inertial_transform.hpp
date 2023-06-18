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

#ifndef TAPROOT_INERTIAL_TRANSFORM_HPP_
#define TAPROOT_INERTIAL_TRANSFORM_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/cross_product.hpp"

#include "frame.hpp"
#include "position.hpp"
#include "transform.hpp"

namespace tap::algorithms::transforms
{
// TODO: somewhat inaccurate name since rotational frames are not inertial
template <const Frame& SOURCE, const Frame& TARGET>
class InertialTransform : protected Transform<SOURCE, TARGET>
{
public:
    InertialTransform(
        const Transform<SOURCE, TARGET> transform,
        float xVel,
        float yVel,
        float zVel,
        float rollVel,
        float pitchVel,
        float yawVel)
        : Transform<SOURCE, TARGET>(transform),
          transVel({xVel, yVel, zVel}),
          angVel({xVel, yVel, zVel})
    {
    }

    InertialTransform(
        const Transform<SOURCE, TARGET> transform,
        const CMSISMat<3, 1>& transVel,
        const CMSISMat<3, 1>& angVel)
        : Transform<SOURCE, TARGET>(transform),
          transVel(transVel),
          angVel(angVel)
    {
    }

    InertialTransform(
        const Transform<SOURCE, TARGET> transform,
        CMSISMat<3, 1>&& transVel,
        CMSISMat<3, 1>&& angVel)
        : Transform<SOURCE, TARGET>(transform),
          transVel(std::move(transVel)),
          angVel(std::move(angVel))
    {
    }

    inline InertialTransform(
        float x,
        float y,
        float z,
        float roll,
        float pitch,
        float yaw,
        float xVel,
        float yVel,
        float zVel,
        float rollVel,
        float pitchVel,
        float yawVel)
        : Transform<SOURCE, TARGET>(x, y, z, roll, pitch, yaw),
          transVel({xVel, yVel, zVel}),
          angVel({rollVel, pitchVel, yawVel})
    {
    }

    using Transform<SOURCE, TARGET>::apply;
    using Transform<SOURCE, TARGET>::getTranslation;
    using Transform<SOURCE, TARGET>::getRotation;
    using Transform<SOURCE, TARGET>::updateTranslation;
    using Transform<SOURCE, TARGET>::updateRotation;

    inline void updateTransVel(Vector<SOURCE> transVel) { this->transVel = transVel.coordinates_; }

    inline void updateTransVel(float x, float y, float z)
    {
        this->transVel = CMSISMat<3, 1>({x, y, z});
    }

    Vector<TARGET> apply(const Position<SOURCE>& position, const Vector<SOURCE>& velocity) const;
    InertialTransform<TARGET, SOURCE> getInverse() const;

    template <const Frame& NEW>
    InertialTransform<SOURCE, NEW> compose(const InertialTransform<TARGET, NEW>& second) const;

private:
    /**
     * Translation differential vector.
     *
     * The velocity of the target frame origin in the source frame.
     */
    CMSISMat<3, 1> transVel;

    /**
     * Differential of rotation matrix.
     *
     * The angular velocity of the target frame coordinates in the source frame.
     */
    CMSISMat<3, 1> angVel;
};  // class InertialTransform

/* BEGIN DEFINITIONS */

// TODO: garbled mess
template <const Frame& SOURCE, const Frame& TARGET>
Vector<TARGET> InertialTransform<SOURCE, TARGET>::apply(
    const Position<SOURCE>& position,
    const Vector<SOURCE>& velocity) const
{
    // TODO: INFINITELY CURSED
    // First add the extra velocities induced by angular/translational velocity then rotate like a
    // vector
    return Transform<SOURCE, TARGET>::apply(
        Vector<SOURCE>(velocity.coordinates_ - transVel - cross(angVel, position.coordinates_)));
}

template <const Frame& SOURCE, const Frame& TARGET>
InertialTransform<TARGET, SOURCE> InertialTransform<SOURCE, TARGET>::getInverse() const
{
    return InertialTransform<TARGET, SOURCE>(
        Transform<SOURCE, TARGET>::getInverse(),
        -transVel,
        -angVel);
}

template <const Frame& SOURCE, const Frame& TARGET>
template <const Frame& NEW>
InertialTransform<SOURCE, NEW> InertialTransform<SOURCE, TARGET>::compose(
    const InertialTransform<TARGET, NEW>& second) const
{
    CMSISMat<3, 1> transVel =
        this->transVel + this->rotation * second.transVel + cross(this->angVel, second.translation);
    CMSISMat<3, 1> angVel = this->transVel + second.transVel;
    return InertialTransform(Transform<SOURCE, TARGET>::compose(second), transVel, angVel);
}

}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_INERTIAL_TRANSFORM_HPP_