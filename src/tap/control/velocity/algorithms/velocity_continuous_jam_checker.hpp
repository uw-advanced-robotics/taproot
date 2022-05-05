/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_VELOCITY_CONTINUOUS_JAM_CHECKER_HPP_
#define TAPROOT_VELOCITY_CONTINUOUS_JAM_CHECKER_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/conditional_timer.hpp"

#include "../interfaces/velocity_setpoint_subsystem.hpp"

namespace tap::control::velocity
{
/**
 * A functor (function object) to be used for velocity subsystem jam detection.
 *
 * Uses a conditional timeout to continuously check if the subsystem's current
 * position is within an acceptable range of the desired position. If it
 * is outside of the acceptable range for too long then the timeout times
 * out and the subsystem is considered jammed.
 */
class VelocityContinuousJamChecker
{
public:
    /**
     * @param[in] velocitySetpointSubsystem: the velocity subsystem to do jam checking on.
     * @param[in] velocitySetpointTolerance: the acceptable velocity (in units / second) between
     * the setpoint and current velocity.
     * @param[in] temporalTolerance: the maximum amount of time the velocity difference between the
     * setpoint and current value can be greater than velocitySetpointTolerance before the velocity
     * setpoint subsystem is to be considered jammed.
     */
    VelocityContinuousJamChecker(
        VelocitySetpointSubsystem* velocitySetpointSubsystem,
        float velocitySetpointTolerance,
        uint32_t temporalTolerance)
        : velocitySetpointSubsystem(velocitySetpointSubsystem),
          velocitySetpointTolerance(velocitySetpointTolerance),
          jamTimeout(temporalTolerance)
    {
    }

    /// Resets the jam timer
    void restart() { jamTimeout.restart(); }

    /**
     * Update subsystem jam detection and check whether subsystem is jammed. If subsystem is within
     * the velocity tolerance of desired value then timer is reset, even if at the time of call it
     * would have expired, as being within tolerance is checked first.
     *
     * @note Should be called once per subsystem refresh (it's like an execute).
     *
     * @return `true` if subsystem is jammed, `false` otherwise.
     */
    inline bool check()
    {
        bool withinTolerance = tap::algorithms::compareFloatClose(
            velocitySetpointSubsystem->getVelocity(),
            velocitySetpointSubsystem->getVelocitySetpoint(),
            velocitySetpointTolerance);
        return jamTimeout.execute(!withinTolerance);
    }

    /// @return the jamming distance tolerance of this jam checker
    inline float getJamSetpointTolerance() const { return velocitySetpointTolerance; }

private:
    VelocitySetpointSubsystem* velocitySetpointSubsystem;
    float velocitySetpointTolerance;
    tap::arch::ConditionalMilliTimer jamTimeout;
};

}  // namespace tap::control::velocity

#endif  // TAPROOT_VELOCITY_CONTINUOUS_JAM_CHECKER_HPP_
