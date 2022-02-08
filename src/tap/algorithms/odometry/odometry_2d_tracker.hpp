/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of taproot.
 *
 * taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with taproot.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef ODOMETRY_2D_TRACKER_HPP_
#define ODOMETRY_2D_TRACKER_HPP_

#include "modm/math/geometry/location_2d.hpp"

#include "odometry_2d_interface.hpp"

namespace tap::algorithms::odometry
{
// Forward declarations
class ChassisWorldYawGetterInterface;
class ChassisDisplacementObserverInterface;

/**
 * Class for tracking the 2D position of an object over time.
 *
 * The faster update() is called the better.
 */
class Odometry2DTracker : public Odometry2DInterface
{
public:
    /**
     * @param chassisYawGetter pointer to an object which implements the
     *      ChassisOrientationGetterInterface. Should return the angle of the chassis
     *      forward vector relative to the x-axis of the field.
     * @param chassisDisplacementGetter pointer to an object which implements the
     *      ChassisDisplacementObserverInterface. Used for getting the chassis displacement
     *
     * @note it is essential that the chassisOrientationGetter and chassisDisplacementGetter
     *      use the same positive z-axis. The getter interfaces should enforce that they
     *      they use positive z-axis is up, but it's worth noting here again.
     */
    Odometry2DTracker(
        ChassisWorldYawGetterInterface* chassisYawGetter,
        ChassisDisplacementObserverInterface* chassisDisplacementGetter);

    /**
     * Run logic and update tracked chassis position. Call frequently for better
     * results.
     */
    void update();

    /**
     * @return the current odometry frame
     * @see OdometryInterface::getCurrentOdometryFrame()
     */
    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

private:
    ChassisWorldYawGetterInterface* chassisYawGetter;
    ChassisDisplacementObserverInterface* chassisDisplacementGetter;
    modm::Location2D<float> location;
};

}  // namespace tap::algorithms::odometry

#endif  // ODOMETRY_2D_TRACKER_HPP_
