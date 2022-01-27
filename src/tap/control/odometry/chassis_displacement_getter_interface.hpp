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

#ifndef CHASSIS_DISPLACEMENT_GETTER_INTERFACE_HPP_
#define CHASSIS_DISPLACEMENT_GETTER_INTERFACE_HPP_

namespace tap::control::odometry
{
/**
 * Interface for getting chassis displacement in chassis frame
 *
 * Each call to getChassisDisplacement should return the chassis displacement
 * in the chassis frame since the last call.
 *
 * While on flat ground, the chassis positive z-axis is defined as "up" (opposite
 * gravity), positive x-axis is defined as the chassis forward vector, and
 * thus consequently the positive y-axis is chassis "left".
 *
 * Unless you use GPS or some other absolute positioning system, it is expected
 * that over long periods the displacement may not be the most accurate. However
 * if the chassis displacement is queried fast and often the accuracy of the
 * accumulated displacement should be better than it would if it were queried
 * at a lower frequently (like any quasi-integration).
 *
 * Getting chassis displacement may fail as implementor chooses to indicate
 * either values are too stale or sensor went offline etc.
 * 
 * @note The RoboMaster robot building specification manual mentions a coordinate
 *      convention for chassis orientations. Their coordinate convention is different
 *      in that they assert positive z as pointing "towards the center of the Earth".
 */
class ChassisDisplacementGetterInterface
{
public:
    /**
     * Returns the chassis displacement in chassis frame since this method was
     * last called. See class comment for more details on axis orientations.
     *
     * @param[out] x destination for x component of chassis displacement in m. 0 if
     *      valid data unavailable.
     * @param[out] y destination for y component of chassis displacement in m. 0 if
     *      valid data unavailable.
     * @param[out] z destination for z component of chassis displacement in m. 0 if
     *      valid data unavailable.
     *
     * @note while valid data being unavailable implies x, y, and z are set to 0, it does not
     *      mean that x, y, z being zero implies that valid data wasn't available.
     *
     * @return `true` if valid chassis displacement data was available, `false` otherwise.
     */
    virtual bool getChassisDisplacement(float* x, float* y, float* z) = 0;
};

}  // namespace tap::control::odometry

#endif  // CHASSIS_DISPLACEMENT_GETTER_INTERFACE_HPP_
