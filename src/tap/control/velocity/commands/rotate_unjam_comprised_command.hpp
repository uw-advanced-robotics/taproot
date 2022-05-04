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

#ifndef TAPROOT_ROTATE_UNJAM_COMPRISED_COMMAND_HPP_
#define TAPROOT_ROTATE_UNJAM_COMPRISED_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"
#include "tap/control/subsystem.hpp"

#include "../interfaces/velocity_setpoint_subsystem.hpp"

namespace tap::control::velocity
{
// Forward declarations
/**
 * A comprised command that combines the unjam and move commands.
 *
 * Assuming no jams occur, this command behaves like a MoveCommand. It will
 * schedule once and end once the target displacement is reached. If it gets
 * jammed while trying to move then the command will then schedule a UnjamCommand.
 * At this point the command will end when the UnjamCommand ends.
 *
 * See `UnjamCommand` and `MoveCommand` for more details on their respective logic.
 */
class RotateUnjamComprisedCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * @param[in] drivers A pointer to the `Drivers` struct.
     */
    RotateUnjamComprisedCommand(
        tap::Drivers &drivers,
        VelocitySetpointSubsystem &subsystem,
        tap::control::Command &rotateCommand,
        tap::control::Command &unjamRotateCommand);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "agitator shoot"; }

protected:
    VelocitySetpointSubsystem &subsystem;

    tap::control::Command &rotateCommand;

    tap::control::Command &unjamCommand;

    bool unjamSequenceCommencing;

    bool agitatorDisconnectFault;
};

}  // namespace tap::control::velocity

#endif  // TAPROOT_ROTATE_UNJAM_COMPRISED_COMMAND_HPP_
