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

#ifndef CHASSIS_WORLD_YAW_GETTER_INTERFACE_HPP_
#define CHASSIS_WORLD_YAW_GETTER_INTERFACE_HPP_

namespace tap::algorithms::odometry
{
/**
 * Object used to get chassis yaw relative to world frame x-axis. Positive
 * angles sweep from field x-axis to field y-axis (right-handed system).
 *
 * Returned angle is from sweeping from field positive x-axis to chassis forward
 * vector.
 *
 * @note Positive z-axis is straight upwards (opposite gravity). The orientation
 *  of the positive x and positive y axes are implementation-defined.
 *
 * Getting chassis orientation may fail as implementor chooses by returning
 * `false` to indicate either values are too stale or sensor went offline etc.
 */
class ChassisWorldYawGetterInterface
{
public:
    /**
     * Get the chassis' "Yaw" in the world frame. Yaw is measured around the z-axis ("up") and
     * sweeps from the positive x-axis of the world frame to the positive x-axis of the chassis.
     * Positive rotation sweeps from world x-axis to world y-axis (standard right-hand rule).
     * Positive z-axis of world frame is guaranteed to be up (opposite gravity).
     * @param[out] yaw destination for chassis yaw in world frame in radians.
     *      Range is in (-pi, pi).
     * @return `true` if valid chassis orientation data was available,
     *      `false` otherwise.
     */
    virtual bool getChassisWorldYaw(float* yaw) = 0;
};

}  // namespace tap::algorithms::odometry

#endif  // CHASSIS_WORLD_YAW_GETTER_INTERFACE_HPP_
