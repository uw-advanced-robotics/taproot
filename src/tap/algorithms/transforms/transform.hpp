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

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"

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
     * Constructs a new Transform, which represents a transformation between two frames.
     *
     * @param rotation Initial rotation of this transformation.
     * @param position Initial position of this transformation.
     */
    Transform(CMSISMat<3, 3>& rotation, CMSISMat<3, 1>& position);

    /**
     * Construct a new Transform, which represents a transformation between two frames.
     * Rotations are implied in order of C->B->A.
     * As an example, for an x-forward + z-down coordinate system,
     * this is in the order of yaw->pitch->roll.
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
     * Returns the composed transformation of the given transformations.
     *
     * @param source Transformation from frame A to frame B.
     * @param target Transformation from frame B to frame C.
     * @return Transformation from frame A to frame C.
     */
    template <typename SRC, typename TARG, typename NEWTARGET>
    static Transform<SRC, NEWTARGET> compose(
        Transform<SRC, TARG>& firstTF,
        Transform<TARG, NEWTARGET>& secondTF);

    /**
     * @return Inverse of this Transform.
     */
    Transform<TARGET, SOURCE> getInverse();

    /**
     * Transforms given position as read by the source frame
     * and computes the equivalent vector components in the target frame's basis.
     *
     * @param pos Position as read by source frame
     * @return Position in target frame's basis.
     */
    CMSISMat<3, 1> applyToPosition(CMSISMat<3, 1>& pos);

    /**
     * Transforms a vector as read by the source frame and computes the equivalent vector
     * components in the target frame's basis. The difference from applyToPosition is that this
     * operation does not alter the magnitude of the components, and just rotates the provided
     * vector.
     *
     * @param vec Vector as read by source frame.
     * @return Vector in target frame's basis.
     */
    CMSISMat<3, 1> applyToVector(CMSISMat<3, 1>& vec);

    /**
     * Updates the rotation of the current transformation matrix.
     *
     * @param newRot updated rotation matrix.
     */
    void updateRotation(CMSISMat<3, 3>& newRot);

    /**
     * Updates the position of the current transformation matrix.
     *
     * @param newPos updated position vector.
     */
    void updatePosition(CMSISMat<3, 1>& newPos);

private:
    /**
     * Rotation matrix.
     */
    CMSISMat<3, 3> rotation;

    /**
     * Position vector.
     */
    CMSISMat<3, 1> position;

    /**
     * Transpose of rotation. Computed and stored at beginning
     * for use in other computations.
     */
    CMSISMat<3, 3> tRotation;
};
}  // namespace tap::algorithms::transforms
#include "transform.cpp"
#endif
