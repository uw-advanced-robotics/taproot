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

#ifndef ODOMETRY_2D_SUBSYSTEM_HPP_
#define ODOMETRY_2D_SUBSYSTEM_HPP_

// Header included instead of forward declared because template uses default args
#include "tap/control/subsystem.hpp"

#include "modm/math/geometry/location_2d.hpp"

#include "odometry_2d_interface.hpp"
#include "odometry_2d_tracker.hpp"

// Forward declarations
namespace tap
{
class Drivers;
namespace control::chassis
{
class ChassisSubsystemInterface;
}
}  // namespace tap

namespace tap::algorithms::odometry
{
// Forward declarations
class ChassisWorldYawGetterInterface;
class ChassisDisplacementObserverInterface;

/**
 * A subsystem for keeping track of the 2D-position of a chassis in the field
 * using chassis displacement reported by a ChassisDisplacementObserverInterface and
 * chassis yaw reported by a ChassisOrientationGetterInterface
 *
 * This system ignores vertical movement.
 *
 * Like almost any controls subsystem, faster refresh rate equals better results.
 */
class Odometry2DSubsystem : public tap::control::Subsystem, public Odometry2DInterface
{
public:
    /**
     * @param drivers pointer to drivers (for registering subsystem)
     * @param chassisOrientationGetter pointer to an object which implements the
     *      ChassisOrientationGetterInterface. Should return the angle of the chassis
     *      forward vector relative to the x-axis of the field.
     * @param chassisDisplacementGetter pointer to an object which implements the
     *      ChassisDisplacementObserverInterface. Used for getting the chassis displacement
     *
     * @note it is essential that the chassisOrientationGetter and chassisDisplacementGetter
     *      use the same positive z-axis. The getter interfaces should enforce that they
     *      they use positive z-axis is up, but it's worth noting here again.
     */
    Odometry2DSubsystem(
        tap::Drivers* drivers,
        ChassisWorldYawGetterInterface* chassisYawGetter,
        ChassisDisplacementObserverInterface* chassisDisplacementGetter)
        : Subsystem(drivers),
          odometryTracker(chassisYawGetter, chassisDisplacementGetter)
    {
    }

    /**
     * Run subsystem logic and update tracked chassis position. Call frequently for better
     * results. The main reason to use this class is for its implementation of this function,
     * hence it doesn't make sense to modify it so it's declared final.
     */
    inline void refresh() final { odometryTracker.update(); }

    /**
     * @return the current odometry frame
     * @see OdometryInterface::getCurrentOdometryFrame()
     */
    inline modm::Location2D<float> getCurrentLocation2D() const final
    {
        return odometryTracker.getCurrentLocation2D();
    }

private:
    Odometry2DTracker odometryTracker;
};

}  // namespace tap::algorithms::odometry

#endif  // ODOMETRY_2D_SUBSYSTEM_HPP_
