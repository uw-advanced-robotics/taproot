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

#ifndef ODOMETRY_SUBSYSTEM_HPP_
#define ODOMETRY_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "odometry_interface.hpp"

// Forward declarations
namespace tap
{
class Drivers;
namespace control::chassis
{
class ChassisSubsystemInterface;
}
}  // namespace tap

namespace tap::control::odometry
{
// Forward declarations
class ChassisOrientationGetterInterface;
class ChassisVelocityGetterInterface;

/**
 * A subsystem for keeping track of the position of a chassis in the field
 * using chassis velocity reported by a ChassisVelocityGetterInterface and
 * chassis orientation reported by a ChassisOrientationGetterInterface
 *
 * Like controls subsystem, a fast and consistent refresh rate is necessary
 * for good output.
 */
class VelocityOdometrySubsystem : public tap::control::Subsystem, public OdometryInterface
{
public:
    /**
     * @param drivers pointer to drivers
     * @param chassisOrientationGetter pointer to an object which implements the
     *      ChassisOrientationGetterInterface. Should return the angle of the chassis
     *      forward vector relative to the x-axis of the field.
     * @param chassisVelocityGetter pointer to an object which implements the
     *      ChassisVelocityGetterInterface. Used for getting the chassis velocity
     */
    VelocityOdometrySubsystem(
        tap::Drivers* drivers,
        ChassisOrientationGetterInterface* chassisOrientationGetter,
        ChassisVelocityGetterInterface* chassisVelocityGetter);

    /**
     * Run subsystem logic and update subsystem position. Call frequently for better results.
     * The main reason to use this class is for its implementation of this function, hence
     * it doesn't make sense to modify it so it's declared final.
     */
    void refresh() final;

private:
    tap::Drivers* drivers;
    ChassisOrientationGetterInterface* chassisOrientationGetter;
    ChassisVelocityGetterInterface* chassisVelocityGetter;
    uint32_t prevTime;
};

}  // namespace tap::control::odometry

#endif  // ODOMETRY_SUBSYSTEM_HPP_
