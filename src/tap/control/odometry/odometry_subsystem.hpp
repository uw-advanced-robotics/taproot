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
class ChassisDisplacementGetterInterface;

/**
 * A subsystem for keeping track of the position of a chassis in the field
 * using chassis displacement reported by a ChassisDisplacementGetterInterface and
 * chassis yaw reported by a ChassisOrientationGetterInterface
 *
 * This system is dumb about vertical movement. It is designed with primarily flat
 * ground in mind and does not use chassis yaw or chassis pitch.
 *
 * Like almost any controls subsystem, faster refresh rate equals better results.
 */
class OdometrySubsystem : public tap::control::Subsystem, public OdometryInterface
{
public:
    /**
     * @param drivers pointer to drivers
     * @param chassisOrientationGetter pointer to an object which implements the
     *      ChassisOrientationGetterInterface. Should return the angle of the chassis
     *      forward vector relative to the x-axis of the field.
     * @param chassisDisplacementGetter pointer to an object which implements the
     *      ChassisDisplacementGetterInterface. Used for getting the chassis displacement
     */
    OdometrySubsystem(
        tap::Drivers* drivers,
        ChassisOrientationGetterInterface* chassisOrientationGetter,
        ChassisDisplacementGetterInterface* chassisDisplacementGetter);

    /**
     * Run subsystem logic and update subsystem position. Call frequently for better results.
     * The main reason to use this class is for its implementation of this function, hence
     * it doesn't make sense to modify it so it's declared final.
     */
    void refresh() final;

    /**
     * @return the current odometry frame
     * @see OdometryInterface::getCurrentOdometryFrame()
     */
    inline const OdometryFrame& getCurrentOdometryFrame() const final;

    /**
     * Set the current position of the robot as the origin.
     * @see OdometryInterface::resetPositionOrigin()
     */
    inline void resetPositionOrigin() final;

private:
    tap::Drivers* drivers;
    ChassisOrientationGetterInterface* chassisOrientationGetter;
    ChassisDisplacementGetterInterface* chassisDisplacementGetter;
    OdometryFrame odometryFrame;
};

}  // namespace tap::control::odometry

#endif  // ODOMETRY_SUBSYSTEM_HPP_
