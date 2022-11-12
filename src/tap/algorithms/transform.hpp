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
#ifndef TAPROOT_TRANSFORMS_HPP_
#define TAPROOT_TRANSFORMS_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/cmsis_mat.hpp"

// TODO:
namespace tap::algorithms
{
/**
 Represents a transformation from one coordinate frame to another.

    A Transform from frame A to frame B defines a relationship between the two frames, such that a
    spatial measurement in frame A can be represented equivalently in frame B by applying a
    translational and rotational offset. This process is known as *applying* a transform.

    Transforms are specified as a translation and rotation of some "target" frame relative to some
    "source" frame. The "translation" is the target frame's origin in source frame, and the
    "rotation" is the target frame's orientation relative to the source frame's orientation.

    Conceptually, translations are applied "before" rotations: a rotation does not affect the origin
    point of the target frame, only rotates the axes of that frame.

    Utilizes arm's CMSIS matrix operations.

    @param SOURCE represents _.
    @param TARGET represents __.
 */
template <typename SOURCE, typename TARGET>
class Transform
{
public:
    /**
     * Constructs a Transform, which represents a transformation between two frames.
     * 
     * NEEDS TO TAKE A ROTATION AND A TRANSLATION..???? rotation matrices and translation vector
     *
     * position x,y,z, and rotation matrix or roll pitch yaw
     * 3 parameters for angle
     * 
     * @param Frame1
     * @param Frame2
     */
Transform(SOURCE Frame1, TARGET Frame2){};

/**
 * Returns the composed transformation of the given frames. 
 * 
 * transform a to b, transform b to c
 * output: a to c
 * 
 * @param Frame1 The SOURCE frame of the composition.
 * @param Frame2 
 * @return Transform over Frame1 and Frame2.
 */
Transform compose(Transform source, Transform target) {};

/**
 * Inverts given transform
 * 
 * @return Transform 
 */
Transform inverse() {}

/**
 * 
 * take in a position as read by the source frame (source frame's vector compoennets) 
 * it computes the vector components in the target frame - in the target frame's basis
 * 
 * takes in position relative to source frame and converts it to be in terms of target frame?
 * 
 * pass in 
 * 
 * quaternions.........make compositions faster........
 * just use rotation matrices bro...
 * 
 * 
 */
applyToPosition() {};

// 
applyToVector() {};

// Represents rotation matrix.
const CMSISMat<2,2> rotation;

// Represents position vector.
vector<vector<vector<int>>> vector_3d(4);

};
}  // namespace tap::algorithms
#endif
