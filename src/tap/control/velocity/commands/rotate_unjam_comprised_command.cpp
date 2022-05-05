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

#include "rotate_unjam_comprised_command.hpp"

#include <cassert>

using namespace tap::control;

namespace tap::control::velocity
{
RotateUnjamComprisedCommand::RotateUnjamComprisedCommand(
    tap::Drivers &drivers,
    VelocitySetpointSubsystem &subsystem,
    RotateCommand &rotateCommand,
    UnjamRotateCommand &unjamRotateCommand)
    : tap::control::ComprisedCommand(&drivers),
      subsystem(subsystem),
      rotateCommand(rotateCommand),
      unjamCommand(unjamCommand),
      unjamSequenceCommencing(false)
{
    assert(
        (1UL << subsystem.getGlobalIdentifier()) == rotateCommand.getRequirementsBitwise() &&
        rotateCommand.getRequirementsBitwise() == unjamCommand.getRequirementsBitwise());

    comprisedCommandScheduler.registerSubsystem(&subsystem);
    addSubsystemRequirement(&subsystem);
}

void RotateUnjamComprisedCommand::initialize()
{
    comprisedCommandScheduler.addCommand(&rotateCommand);
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
        comprisedCommandScheduler.addCommand(&unjamCommand);
    }
    comprisedCommandScheduler.run();
}

void RotateUnjamComprisedCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&unjamCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&rotateCommand, interrupted);
}

bool RotateUnjamComprisedCommand::isFinished() const
{
    return (!unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&rotateCommand)) ||
           (unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&unjamCommand));
}

}  // namespace tap::control::velocity
