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

#ifndef CHASSIS_VELOCITY_GETTER_INTERFACE_HPP_
#define CHASSIS_VELOCITY_GETTER_INTERFACE_HPP_

namespace tap::control::odometry
{
/**
 * Interface for getting chassis velocity in chassis frame
 *
 * Important that positive z-axis that this uses points in the same direction
 * as the positive z-axis which the orientation getter uses (and that they both
 * are the same handedness, but why would you ever use left-handed axes?!).
 *
 * Getting chassis velocity may fail as implementor chooses to indicate
 * either values are too stale or sensor went offline etc.
 * If return value is `false` odometry logic won't run that tick.
 */
class ChassisVelocityGetterInterface
{
public:
    /**
     * @param[out] x destination for x component of chassis velocity in m/s. 0 if
     *      valid data unavailable.
     * @param[out] y destination for y component of chassis velocity in m/s. 0 if
     *      valid data unavailable.
     * @return `true` if valid chassis velocity data was available, `false` otherwise.
     */
    virtual bool getChassisVelocity(float* x, float* y) = 0;
};

}  // namespace tap::control::odometry

#endif  // CHASSIS_VELOCITY_GETTER_INTERFACE_HPP_
