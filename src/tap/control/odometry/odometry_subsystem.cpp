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

#include "odometry_subsystem.hpp"

#include <cmath>

#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/drivers.hpp"

#include "chassis_displacement_getter_interface.hpp"
#include "chassis_orientation_getter_interface.hpp"

namespace tap::control::odometry
{
OdometrySubsystem::OdometrySubsystem(
    tap::Drivers* drivers,
    ChassisOrientationGetterInterface* chassisOrientationGetter,
    ChassisDisplacementGetterInterface* chassisDisplacementGetter)
    : Subsystem(drivers),
      drivers(drivers),
      chassisOrientationGetter(chassisOrientationGetter),
      chassisDisplacementGetter(chassisDisplacementGetter)
{
}

void OdometrySubsystem::refresh()
{
    float dxChassisRelative = 0.0f;
    float dyChassisRelative = 0.0f;
    float dzChassisRelative = 0.0f;
    float chassisAngle = 0.0f;

    bool validDisplacementAvailable = chassisDisplacementGetter->getChassisDisplacement(
        &dxChassisRelative,
        &dyChassisRelative,
        &dzChassisRelative);
    bool validOrientationAvailable = chassisOrientationGetter->getChassisYaw(&chassisAngle);

    // Only execute logic if velocity and orientation were available
    if (validDisplacementAvailable && validOrientationAvailable)
    {
        const float sinTheta = sinf(chassisAngle);
        const float cosTheta = cosf(chassisAngle);
        odometryFrame.x += dxChassisRelative * cosTheta - dyChassisRelative * sinTheta;
        odometryFrame.y += dxChassisRelative * sinTheta + dyChassisRelative * cosTheta;
        odometryFrame.z += dzChassisRelative;
    }
}

inline const OdometryFrame& OdometrySubsystem::getCurrentOdometryFrame() const
{
    return odometryFrame;
}

inline void OdometrySubsystem::resetPositionOrigin()
{
    odometryFrame.x = 0.0f;
    odometryFrame.y = 0.0f;
    odometryFrame.z = 0.0f;
}

}  // namespace tap::control::odometry
