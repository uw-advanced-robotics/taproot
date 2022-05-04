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

#ifndef TAPROOT_ROTATE_COMMAND_HPP_
#define TAPROOT_ROTATE_COMMAND_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "../interfaces/velocity_setpoint_subsystem.hpp"
#include "modm/math/filter/pid.hpp"

namespace tap::control::velocity
{
/**
 * Displaces the connected subsystem some value in some desired time. Currently
 * pass in a displacement and time to move and it uses `tap::arch::getTimeMilliseconds()`
 * to determine the speed to move at.
 *
 * Ends if subsystem is offline.
 */
class RotateCommand : public tap::control::Command
{
public:
    /**
     * @param[in] targetDisplacement The desired change in subsystem value in subsystem units.
     * @param[in] setpointTolerance The difference between current and desired value when the
     *      command will be considered to be completed (used in the `isFinished` function). Uses
     *      the same units as the subsystem's setpoint.
     */
    struct Config
    {
        float targetDisplacement;
        float desiredVelocity;
        float velocitySetpointTolerance;
    };

    /**
     * @param[in] velocitySetpointSubsystem The subsystem associated with the rotate command.
     * @attention the ramp value is calculated by finding the rotation speed
     *      (\f$targetDisplacement / moveTime\f$), and then multiplying this by
     *      the period (how often the ramp is called)
     */
    RotateCommand(VelocitySetpointSubsystem& velocitySetpointSubsystem, const Config& config);

    const char* getName() const override { return "rotate command"; }

    bool isReady() override
    {
        return !velocitySetpointSubsystem.isJammed() && velocitySetpointSubsystem.isOnline();
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    Config config;

    VelocitySetpointSubsystem& velocitySetpointSubsystem;

    float finalTargetPosition = 0;

    bool withinSetpointTolerance() const;
};  // class RotateCommand

}  // namespace tap::control::velocity

#endif  // TAPROOT_ROTATE_COMMAND_HPP_
