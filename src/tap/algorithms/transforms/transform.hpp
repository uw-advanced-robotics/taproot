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
#ifndef TAPROOT_TRANSFORM_HPP_
#define TAPROOT_TRANSFORM_HPP_

#include "tap/algorithms/math_user_utils.hpp"

#include "orientation.hpp"
#include "position.hpp"
#include "vector.hpp"

namespace tap::algorithms::transforms
{
/**
 Represents a transformation from one coordinate frame to another.

    A Transform from frame A to frame B defines a relationship between the two frames, such that a
    spatial measurement in frame A can be represented equivalently in frame B by applying a
    translational and rotational offset. This process is known as *applying* a transform.

    Transforms are specified as a translation and rotation of some "target" frame relative to some
    "source" frame. The "translation" is the target frame's origin in source frame, and the
    "rotation" is the target frame's orientation relative to the source frame's orientation.

    Conceptually, translations are applied "before" rotations. This means that the origin of the
    target frame is entirely defined by the translation in the source frame, and the rotation serves
    only to change the orientation of the target frame's axes relative to the source frame.

    Utilizes arm's CMSIS matrix operations.

    @param SOURCE represents the source frame of the transformation.
    @param TARGET represents the target frame of the transformation.
 */
class Transform
{
    friend class Transform;
public:
    /**
     * @param rotation Initial rotation of this transformation.
     * @param position Initial translation of this transformation.
     */
    Transform(const Position& translation, const Orientation<SOURCE>& rotation);
    Transform(Position&& translation, Orientation<SOURCE>&& rotation);

    /**
     * @param rotation Initial rotation of this transformation.
     * @param position Initial translation of this transformation.
     */
    Transform(const CMSISMat<3, 1>& translation, const CMSISMat<3, 3>& rotation);
    Transform(CMSISMat<3, 1>&& translation, CMSISMat<3, 3>&& rotation);

    /**
     * Constructs rotations using XYZ Euler angles,
     * so rotations are applied in order of A, B, then C.
     * As an example, for an x-forward, z-up coordinate system,
     * this is in the order of roll, pitch, then yaw.
     *
     * @param x: Initial x-component of the translation.
     * @param y: Initial y-component of the translation.
     * @param z: Initial z-component of the translation.
     * @param A: Initial rotation angle about the x-axis.
     * @param B: Initial rotation angle about the y-axis.
     * @param C: Initial rotation angle about the z-axis.
     */
    Transform(float x, float y, float z, float roll, float pitch, float yaw);

    // TODO: template specialization for transform between identical frames??
    /**
     * Constructs an identity transform.
     */
    static inline Transform identity() { return Transform(0., 0., 0., 0., 0., 0.); }

    /**
     * Apply this transform to a position.
     *
     * @param[in] position Position in source frame.
     * @return Position in target frame.
     */
    Position apply(const Position& position) const;

    /**
     * Rotates a vector in the source frame to a vector in the target frame.
     *
     * Intended to be used for things like velocities and accelerations which represent the
     * difference between two positions in space, since both positions get translated the same way,
     * causing the translation to cancel out.
     *
     * @param vector Vector as read by source frame.
     * @return Vector in target frame's basis.
     */
    Vector apply(const Vector& vector) const;

    /**
     *
     */
    Orientation apply(const Orientation& orientation) const;

    /**
     * Updates the translation of the current transformation matrix.
     *
     * @param newTranslation updated position of target in source frame.
     */
    inline void updateTranslation(const Position& newTranslation)
    {
        this->translation = newTranslation.coordinates_;
    }

    inline void updateTranslation(Position&& newTranslation)
    {
        this->translation = std::move(newTranslation.coordinates_);
    }

    /**
     * Updates the translation of the current transformation matrix.
     *
     * @param x new translation x-component.
     * @param y new translation y-component.
     * @param z new translation z-component.
     */
    inline void updateTranslation(float x, float y, float z)
    {
        this->translation = CMSISMat<3, 1>({x, y, z});
    }

    /**
     * Updates the rotation of the current transformation matrix.
     *
     * @param newRotation updated orienation of target frame in source frame.
     */
    inline void updateRotation(const Orientation& newRotation)
    {
        this->rotation = newRotation.matrix_;
        this->tRotation = this->rotation.transpose();
    }

    inline void updateRotation(Orientation&& newRotation)
    {
        this->rotation = std::move(newRotation.matrix_);
        this->tRotation = this->rotation.transpose();
    }

    /**
     * Updates the rotation of the current transformation matrix.
     * Takes rotation angles in the order of roll->pitch->yaw.
     *
     * @param roll updated rotation angle about the x-axis.
     * @param pitch updated rotation angle about the y-axis.
     * @param yaw updated rotation angle about the z-axis.
     */
    void updateRotation(float roll, float pitch, float yaw)
    {
        this->rotation = Orientation(roll, pitch, yaw).matrix_;
        this->tRotation = this->rotation.transpose();
    }

    /**
     * @return Inverse of this Transform.
     */
    Transform getInverse() const;

    /**
     * Returns the composed transformation of the given transformations.
     * @return Transformation from frame A to frame C.
     */
    Transform compose(const Transform& second) const;

    /* Getters */
    inline Position getTranslation() const { return Position(translation); };

    inline Orientation getRotation() const { return Orientation(rotation); }

    /**
     * Get the roll of this transformation
     */
    float getRoll() const;

    /**
     * Get the pitch of this transformation
     */
    float getPitch() const;

    /**
     * Get the pitch of this transformation
     */
    float getYaw() const;

    /**
     * Get the x-component of this transform's translation
     */
    float getX() const;

    /**
     * Get the x-component of this transform's translation
     */
    float getY() const;

    /**
     * Get the x-component of this transform's translation
     */
    float getZ() const;

private:
    /**
     * Translation vector.
     */
    CMSISMat<3, 1> translation;

    /**
     * Rotation matrix.
     */
    CMSISMat<3, 3> rotation;

    /**
     * Transpose of rotation matrix. Computed and stored at beginning
     * for use in other computations.
     *
     * The transpose of a rotation is its inverse.
     */
    CMSISMat<3, 3> tRotation;
};  // class Transform

/* Begin definitions */
inline Transform::Transform(const Position& translation, const Orientation& rotation)
    : translation(translation.coordinates_),
      rotation(rotation.matrix_),
      tRotation(rotation.matrix_.transpose())
{
}

inline Transform::Transform(Position&& translation, Orientation&& rotation)
    : translation(std::move(translation.coordinates_)),
      rotation(std::move(rotation.matrix_)),
      tRotation(rotation.matrix_.transpose())
{
}

inline Transform::Transform(const CMSISMat<3, 1>& translation, const CMSISMat<3, 3>& rotation)
    : translation(translation),
      rotation(rotation),
      tRotation(rotation.transpose())
{
}

template <const Frame& SOURCE, const Frame& TARGET>
inline Transform<SOURCE, TARGET>::Transform(CMSISMat<3, 1>&& translation, CMSISMat<3, 3>&& rotation)
    : translation(std::move(translation)),
      rotation(std::move(rotation)),
      tRotation(rotation.transpose())
{
}

Transform::Transform(float x, float y, float z, float roll, float pitch, float yaw)
    : translation({x, y, z}),
      rotation(fromEulerAngles(roll, pitch, yaw)),
      tRotation(rotation.transpose())
{
}

Position Transform::apply(const Position<SOURCE>& position) const
{
    return Position(tRotation * (position.coordinates_ - translation));
}

Vector Transform::apply(const Vector& vector) const
{
    return Vector(tRotation * vector.coordinates_);
}

Orientation Transform::apply(const Orientation& orientation) const
{
    return Orientation(tRotation * orientation.coordinates_);
}

Transform Transform::getInverse() const
{
    // negative transposed rotation matrix times original position = new position
    CMSISMat<3, 1> invTranslation = tRotation * translation;
    invTranslation = -invTranslation;
    return Transform(invTranslation, tRotation);
}

Transform Transform::compose(
    const Transform& second) const
{
    CMSISMat<3, 3> newRot = this->rotation * second.rotation;
    CMSISMat<3, 1> newPos = this->translation + this->rotation * second.translation;
    return Transform(newPos, newRot);
}
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_TRANSFORM_HPP_
