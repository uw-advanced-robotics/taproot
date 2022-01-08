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
 * Struct for holding the position in meters and yaw and pitch (in radians)
 * of a chassis relative to some coordinate frame.
 */
struct OdometryFrame
{
public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float chassisYaw = 0.0f;
    float chassisPitch = 0.0f;
};

class OdometryInterface
{
public:
    /**
     * @param[out] output destination for the current odometry frame
     */
    inline void getCurrentOdometryFrame(OdometryFrame* output)
    {
        *output = odometryFrame;
    }

    /**
     * Resets the stored odometry frame such that the current position of
     * the chassis is considered the origin.
     */
    inline void resetPositionOrigin()
    {
        odometryFrame.x = 0.0f;
        odometryFrame.y = 0.0f;
        odometryFrame.z = 0.0f;
    }

protected:
    OdometryFrame odometryFrame;
};

}  // namespace tap::control::odometry

#endif  // ODOMETRY_SUBSYSTEM_INTERFACE_HPP_
