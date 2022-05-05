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

#ifndef TAPROOT_ROTATE_UNJAM_COMPRISED_COMMAND_HPP_
#define TAPROOT_ROTATE_UNJAM_COMPRISED_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "../interfaces/velocity_setpoint_subsystem.hpp"

#include "rotate_command.hpp"
#include "unjam_rotate_command.hpp"

namespace tap::control::velocity
{
/**
 * A comprised command that combines the unjam and rotate commands. Will rotate the agitator forward
 * when not jammed by scheduling the rotate command. When jammed will schedule the unjam rotate
 * command. Once the unjam command is scheduled, this command will be unscheduled when the unjam
 * command finishes
 *
 * @see UnjamRotateCommand
 * @see RotateCommand
 */
class RotateUnjamComprisedCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * @param[in] drivers A reference to the `Drivers` struct.
     * @param[in] subsystem The VelocitySetpointSubsystem that will be rotated or unjammed.
     * @param[in] rotateCommand A command that rotates the agitator forward.
     * @param[in] unjamRotateCommand A command that unjams the agitator.
     *
     * @note The rotate and unjam rotate commands must have the same subsystem requirement. This
     * subsystem requirement must be the subsystem passed in.
     */
    RotateUnjamComprisedCommand(
        tap::Drivers &drivers,
        VelocitySetpointSubsystem &subsystem,
        RotateCommand &rotateCommand,
        UnjamRotateCommand &unjamRotateCommand);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "rotate unjam cc"; }

protected:
    VelocitySetpointSubsystem &subsystem;

    RotateCommand &rotateCommand;

    UnjamRotateCommand &unjamCommand;

    /// True if the agitator is being unjammed
    bool unjamSequenceCommencing;
};

}  // namespace tap::control::velocity

#endif  // TAPROOT_ROTATE_UNJAM_COMPRISED_COMMAND_HPP_
