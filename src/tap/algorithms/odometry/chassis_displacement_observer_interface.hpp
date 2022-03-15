/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_CHASSIS_DISPLACEMENT_OBSERVER_INTERFACE_HPP_
#define TAPROOT_CHASSIS_DISPLACEMENT_OBSERVER_INTERFACE_HPP_

#include "modm/math/geometry/vector.hpp"

namespace tap::algorithms::odometry
{
/**
 * Interface for getting chassis displacement in chassis frame
 *
 * Implementations of `getChassisDisplacement` should return the absolute chassis
 * displacement in the chassis frame since some arbitrary time at startup. Note
 * this value does not have a useful physical interpretation (e.g.: a robot that
 * drives a complete circle going forward and returns to it's original point will
 * have a positive absolute x-displacement in chassis frame corresponding to the
 * circle's circumference).
 *
 * This absolute value return was chosen over a volatile getter that returned
 * displacement since last call to avoid having getter modify the state of the class (and
 * thus avoid concerns about multiple consumers sharing this object)
 *
 * To generate useful measurements in a non-rotating reference frame from this getter
 * users should differentiate this returned value by sampling this value frequently,
 * finding the delta position, and orienting that displacement so that it matches
 * the reference frame, then adding that displacement to the tracked chassis position.
 *
 * While on flat ground, the chassis positive z-axis is defined as "up" (opposite
 * gravity), positive x-axis is defined as the chassis forward vector, and
 * thus consequently the positive y-axis is chassis "left".
 *
 * Getting chassis displacement may fail as implementor chooses to indicate
 * either values are too stale or sensor went offline etc.
 *
 * Most implementations probably will not support getting z-displacement, and in
 * that case should just return 0 for z-displacement.
 *
 * @note The RoboMaster robot building specification manual mentions a coordinate
 *      convention for chassis orientations. Their coordinate convention is different
 *      in that they assert positive z as pointing "towards the center of the Earth".
 */
class ChassisDisplacementObserverInterface
{
public:
    /**
     * Returns the chassis displacement in chassis frame since some fixed arbitrary
     * point in time near startup.
     *
     * @param[out] displacement if valid data is available the x, y, and z of this pointed
     *      to vector will be populated with the appropriate absolute displacement in implementation
     *      specific units.
     *
     * @return `true` if valid chassis displacement data was available, `false` otherwise.
     */
    virtual bool getChassisDisplacement(modm::Vector<float, 3>* const displacement) const = 0;
};

}  // namespace tap::algorithms::odometry

#endif  // TAPROOT_CHASSIS_DISPLACEMENT_OBSERVER_INTERFACE_HPP_
