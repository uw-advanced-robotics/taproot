/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/control/subsystem.hpp"

#include "modm/math/geometry/location_2d.hpp"

#include "otto_chassis_velocity_displacement_2d_observer.hpp"
#include "otto_chassis_world_yaw_observer.hpp"

namespace tap::algorithms::odometry
{
/**
 * Provides a subsystem interface wrapper around an OdometryTracker object
 * 
 * @see OdometryTracker
 */
template <typename DisplacementObserver, typename YawObserver>
class OdometrySubsystem : public Odometry2DInterface
{
public:
    OdometrySubsystem()
};

}  // namespace tap::algorithms::odometry

#endif  // ODOMETRY_2D_SUBSYSTEM_HPP_
