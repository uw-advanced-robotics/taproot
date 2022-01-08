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
#include "mecanum_velocity_odometry_subsystem.hpp"

#include <cmath>

#include "tap/architecture/clock.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/drivers.hpp"

namespace tap::control::odometry
{
MecanumVelocityOdometrySubsystem::MecanumVelocityOdometrySubsystem(
    tap::Drivers* drivers,
    ChassisOrientationGetterInterface* chassisOrientationGetter,
    ChassisVelocityGetterInterface* chassisVelocityGetter)
    : Subsystem(drivers),
      drivers(drivers),
      chassisOrientationGetter(chassisOrientationGetter),
      chassisVelocityGetter(chassisVelocityGetter)
{
}

void MecanumVelocityOdometrySubsystem::refresh()
{
    // In the future it may be possible to just grab the scheduler tick
    // frequency if we had a freeRTOS like config file. Currently the subsystem
    // has no way of knowing the scheduler frequency though, so we get it
    // ourselves. Calculate the dt outside of the if statement to not let
    // dt grow way too large while chassis offline.
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    float xVelocity;
    float yVelocity;
    float chassisAngle;
    bool validVelocityAvailable = chassisVelocityGetter->getChassisVelocity(&xVelocity, &yVelocity);
    bool validOrientationAvailable = chassisOrientationGetter->getChassisOrientation(&chassisAngle);

    // Only execute logic if velocity and orientation were available
    if (validVelocityAvailable && validOrientationAvailable)
    {
        // m/s * ms * (1s / 1000ms) = m
        float dxChassisRelative = xVelocity * dt / 1000.0f;
        float dyChassisRelative = yVelocity * dt / 1000.0f;

        const float sinTheta = sinf(chassisAngle);
        const float cosTheta = cosf(chassisAngle);
        odometryFrame.x += dxChassisRelative * cosTheta - dyChassisRelative * sinTheta;
        odometryFrame.y += dxChassisRelative * sinTheta + dyChassisRelative * cosTheta;
    }
}

}  // namespace tap::control::odometry
