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

#include "agitator_shoot_comprised_command.hpp"

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/control/command_scheduler.hpp>

#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{
namespace agitator
{
ShootComprisedCommand::ShootComprisedCommand(
    aruwlib::Drivers* drivers,
    AgitatorSubsystem* agitator,
    float agitatorChangeAngle,
    float maxUnjamAngle,
    uint32_t agitatorRotateTime,
    uint32_t agitatorPauseAfterRotateTime)
    : aruwlib::control::ComprisedCommand(drivers),
      connectedAgitator(agitator),
      agitatorRotateCommand(
          agitator,
          agitatorChangeAngle,
          agitatorRotateTime,
          agitatorPauseAfterRotateTime,
          false),
      agitatorUnjamCommand(agitator, maxUnjamAngle),
      unjamSequenceCommencing(false),
      agitatorDisconnectFault(false)
{
    this->comprisedCommandScheduler.registerSubsystem(agitator);
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(agitator));
}

void ShootComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&agitatorRotateCommand));
    unjamSequenceCommencing = false;
}

void ShootComprisedCommand::execute()
{
    // If agitator has disconnected, set flag to remember this when isFinished() and end()
    // are called. (This check can't be done in isFinished() because it's const function)
    if (!connectedAgitator->isAgitatorOnline())
    {
        agitatorDisconnectFault = true;
    }
    else
    {
        // If agitator isn't disconnected run our normal logic
        if (connectedAgitator->isAgitatorJammed() && !unjamSequenceCommencing)
        {
            // when the agitator is jammed, add the agitatorUnjamCommand
            // the to scheduler. The rotate forward command will be automatically
            // unscheduled.
            unjamSequenceCommencing = true;
            this->comprisedCommandScheduler.addCommand(
                dynamic_cast<Command*>(&agitatorUnjamCommand));
        }
        this->comprisedCommandScheduler.run();
    }
}

void ShootComprisedCommand::end(bool interrupted)
{
    // The command could have also been interrupted by loss of agitator
    // connection. Account for that by OR'ing them.
    interrupted |= agitatorDisconnectFault;
    // agitatorDisconnect has been acknowledged, regardless of previous state
    // set it back to false
    agitatorDisconnectFault = false;

    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&agitatorUnjamCommand),
        interrupted);
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&agitatorRotateCommand),
        interrupted);
}

bool ShootComprisedCommand::isFinished() const
{
    return (!unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&agitatorRotateCommand)) ||
           (unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&agitatorUnjamCommand)) ||
           agitatorDisconnectFault;
}

}  // namespace agitator

}  // namespace aruwsrc
