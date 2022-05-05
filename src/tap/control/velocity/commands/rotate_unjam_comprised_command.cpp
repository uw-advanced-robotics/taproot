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

#include "rotate_unjam_comprised_command.hpp"

using namespace tap::control;

namespace tap::control::velocity
{
RotateUnjamComprisedCommand::RotateUnjamComprisedCommand(
    tap::Drivers &drivers,
    VelocitySetpointSubsystem &subsystem,
    tap::control::Command &rotateCommand,
    tap::control::Command &unjamCommand)
    : tap::control::ComprisedCommand(&drivers),
      subsystem(subsystem),
      rotateCommand(rotateCommand),
      unjamCommand(unjamCommand),
      unjamSequenceCommencing(false),
      agitatorDisconnectFault(false)
{
    this->comprisedCommandScheduler.registerSubsystem(&subsystem);
    this->addSubsystemRequirement(&subsystem);
}

void RotateUnjamComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(&rotateCommand);
    unjamSequenceCommencing = false;
}

void RotateUnjamComprisedCommand::execute()
{
    // If setpointSubsystem isn't disconnected run our normal logic
    if (subsystem.isJammed() && !unjamSequenceCommencing)
    {
        // when the setpointSubsystem is jammed, add the unjamCommand
        // the to scheduler. The rotate forward command will be automatically
        // unscheduled.
        unjamSequenceCommencing = true;
        this->comprisedCommandScheduler.addCommand(&unjamCommand);
    }
    this->comprisedCommandScheduler.run();
}

void RotateUnjamComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&unjamCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&rotateCommand, interrupted);
}

bool RotateUnjamComprisedCommand::isFinished() const
{
    return (!unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&rotateCommand)) ||
           (unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&unjamCommand)) ||
           agitatorDisconnectFault;
}

}  // namespace tap::control::velocity
