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
#include "position.hpp"
#include "pose.hpp"
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
template <typename SOURCE, typename TARGET>
class Transform
{
public:
    /**
     * @param rotation Initial rotation of this transformation.
     * @param position Initial translation of this transformation.
     * 
     * @note input matrices are non-const due to CMSISMat move semantics
     */
    Transform(CMSISMat<3, 3>& rotation, CMSISMat<3, 1>& translation);

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
    Transform(float x, float y, float z, float A, float B, float C);

    /**
     * Constructs an identity transform.
     */
    Transform()
    {
        *this = Transform(0., 0., 0., 0., 0., 0.);
    }

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

    /**
     * Apply this transform to a position.
     * 
     * @param[in] position Position in source frame.
     * @return Position in target frame.
    */
    Position<TARGET> apply(Position<SOURCE>& position);

    /**
     * Apply this transform to a pose.
     * 
     * @param[in] pose Pose in source frame.
     * @return Pose in target frame.
    */
    Pose<TARGET> apply(Pose<SOURCE>& pose);

    /**
     * Rotates a vector in the source frame to a vector in the target frame.
     * 
     * Intended to be used for things like velocities and accelerations which represent the difference
     * between two positions in space, since both positions get translated the same way, causing
     * the translation to cancel out.
     *
     * @param vec Vector as read by source frame.
     * @return Vector in target frame's basis.
     */
    Vector<TARGET> apply(Vector<SOURCE>& vector);

    /**
     * @return Inverse of this Transform.
     */
    Transform<TARGET, SOURCE> getInverse() const;

    inline CMSISMat<3, 1> getTranslation() { return translation; };

    inline CMSISMat<3, 3> getRotation() { return rotation; }

    /**
     * Updates the rotation of the current transformation matrix.
     *
     * @param newRot updated rotation matrix.
     * 
     * @note newRot is non-const due to CMSISMat move semantics
     */
    void updateRotation(CMSISMat<3, 3>& newRot);

    /**
     * Updates the rotation of the current transformation matrix.
     * Takes rotation angles in the order of roll->pitch->yaw.
     *
     * @param A updated rotation angle about the x-axis.
     * @param B updated rotation angle about the y-axis.
     * @param C updated rotation angle about the z-axis.
     */
    void updateRotation(float A, float B, float C)
    {
        updateRotation(rotationMatrix(A, B, C));
    }

    /**
     * Updates the translation of the current transformation matrix.
     *
     * @param newTranslation updated translation vector.
     * 
     * @note input newTranslation is non-const due to CMSISMat move semantics
     */
    inline void updateTranslation(const CMSISMat<3, 1>& newTranslation)
    {
        this->position = std::move(newTranslation);
    }

    /**
     * Updates the position of the current transformation matrix.
     *
     * @param x new translation x-component.
     * @param y new translation y-component.
     * @param z new translation z-component.
     */
    void updateTranslation(float x, float y, float z);

private:
    /**
     * Rotation matrix.
     */
    CMSISMat<3, 3> rotation;

    /**
     * Translation vector.
     */
    CMSISMat<3, 1> translation;

    /**
     * Transpose of rotation. Computed and stored at beginning
     * for use in other computations.
     * 
     * The transpose of a rotation is its inverse.
     */
    CMSISMat<3, 3> tRotation;
};

/**
 * Return a rotation matrix with rotation order A->B->C (roll->pitch->yaw).
*/
CMSISMat<3, 3> rotationMatrix(const float A, const float B, const float C)
{
    return CMSISMat<3,3>({
        cosf(C) * cosf(B),
        (cosf(C) * sinf(B) * sinf(A)) - (sinf(C) * cosf(A)),
        (cosf(C) * sinf(B) * cosf(A)) + sinf(C) * sinf(A),
        sinf(C) * cosf(B),
        sinf(C) * sinf(B) * sinf(A) + cosf(C) * cosf(A),
        sinf(C) * sinf(B) * cosf(A) - cosf(C) * sinf(A),
        -sinf(B),
        cosf(B) * sinf(A),
        cosf(B) * cosf(A)
    });
}

/**
 * Returns the composed transformation of the given transformations.
 *
 * @param source Transformation from frame A to frame B.
 * @param target Transformation from frame B to frame C.
 * @return Transformation from frame A to frame C.
 */
template <typename A, typename B, typename C>
Transform<A, C> compose(const Transform<A, B>& first, const Transform<B, C>& second);

}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_TRANSFORM_HPP_
