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

#include "odometry_2d_subsystem.hpp"

#include <cmath>

#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/drivers.hpp"

#include "modm/math/geometry/angle.hpp"

#include "chassis_displacement_getter_interface.hpp"
#include "chassis_world_yaw_getter_interface.hpp"

namespace tap::control::odometry
{
Odometry2DSubsystem::Odometry2DSubsystem(
    tap::Drivers* drivers,
    ChassisWorldYawGetterInterface* chassisYawGetter,
    ChassisDisplacementGetterInterface* chassisDisplacementGetter)
    : Subsystem(drivers),
      chassisYawGetter(chassisYawGetter),
      chassisDisplacementGetter(chassisDisplacementGetter),
      location(0.0f, 0.0f, 0.0f)
{
}

void Odometry2DSubsystem::refresh()
{
    float dxChassisRelative = 0.0f;
    float dyChassisRelative = 0.0f;
    float dzChassisRelative = 0.0f;
    float chassisAngle = 0.0f;

    bool validDisplacementAvailable = chassisDisplacementGetter->getChassisDisplacement(
        &dxChassisRelative,
        &dyChassisRelative,
        &dzChassisRelative);
    bool validOrientationAvailable = chassisYawGetter->getChassisWorldYaw(&chassisAngle);

    // Only execute logic if velocity and orientation were available
    if (validDisplacementAvailable && validOrientationAvailable)
    {
        // Spec for `Location2D` seems to suggest it should only use normalized angles.
        // chassisYawGetter is specified to return normalized angles
        float worldRelativeOrientation = chassisAngle;
        location.setOrientation(worldRelativeOrientation);
        location.move(modm::Vector2f(dxChassisRelative, dyChassisRelative));
    }
}

}  // namespace tap::control::odometry
