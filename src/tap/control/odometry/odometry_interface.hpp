/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ODOMETRY_INTERFACE_HPP_
#define ODOMETRY_INTERFACE_HPP_

namespace tap::control::odometry
{
/**
 * Struct for holding the position of a chassis in 3D-space. Positive z-axis should
 * be "up" (opposite gravity). No guarantees are given on the direction of the
 * positive x and y axes or on the position of the origin. Right handed coordinate
 * system. Units in meters.
 */
struct OdometryFrame
{
public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/**
 * Interface for retrieving the position of a robot on the field.
 *
 * The coordinate axes are right-handed, and the positive z-axis is guaranteed
 * to be up and away from the floor (opposite gravity). The orientations of
 * the positive x and y axes are not specified by this interface.
 */
class OdometryInterface
{
public:
    /**
     * @return the current odometry frame (the current x, y, z coordinates of the robot)
     */
    virtual inline const OdometryFrame& getCurrentOdometryFrame() const = 0;

    /**
     * Calibrate the interface such that the current position of the robot
     * is considered the origin i.e.: current (x, y, z) = (0, 0, 0)
     *
     * This does not affect the rotation (yaw and pitch) of the reference frame.
     */
    virtual inline void resetPositionOrigin() = 0;
};

}  // namespace tap::control::odometry

#endif  // ODOMETRY_SUBSYSTEM_INTERFACE_HPP_
