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

#ifndef TAPROOT_ROTATE_COMMAND_HPP_
#define TAPROOT_ROTATE_COMMAND_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/command.hpp"

#include "../interfaces/velocity_setpoint_subsystem.hpp"

namespace tap::control::velocity
{
/**
 * A command that rotates the connected subsystem at some speed for some distance.
 *
 * Ends if the subsystem is offline or jammed.
 */
class RotateCommand : public tap::control::Command
{
public:
    /// Config struct that the user passes into the RotateCommand's constructor.
    struct Config
    {
        /// The desired change in units.
        float targetDisplacement;
        /// The desired velocity in units/second
        float desiredVelocity;
        /// The difference between the current and desired value when the command will be considered
        /// to be complete, in units.
        float velocitySetpointTolerance;
    };

    /**
     * @param[in] velocitySetpointSubsystem The subsystem associated with the rotate command.
     * @param[in] config The rotate command's config struct, @see Config.
     */
    RotateCommand(VelocitySetpointSubsystem& velocitySetpointSubsystem, const Config& config);

    const char* getName() const override { return "rotate command"; }

    bool isReady() override
    {
        return !velocitySetpointSubsystem.isJammed() && velocitySetpointSubsystem.isOnline();
    }

    void initialize() override;

    void execute() override {}

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    Config config;

    VelocitySetpointSubsystem& velocitySetpointSubsystem;

    float finalTargetPosition = 0;

    /// @return True if the difference between the setpoint and target is less than the setpoint
    /// tolerance.
    bool withinSetpointTolerance() const
    {
        return algorithms::compareFloatClose(
            velocitySetpointSubsystem.getPosition(),
            finalTargetPosition,
            config.velocitySetpointTolerance);
    }
};  // class RotateCommand

}  // namespace tap::control::velocity

#endif  // TAPROOT_ROTATE_COMMAND_HPP_
