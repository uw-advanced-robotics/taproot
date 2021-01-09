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
    uint32_t agitatorDesiredRotateTime,
    uint32_t minAgitatorRotateTime)
    : aruwlib::control::ComprisedCommand(drivers),
      connectedAgitator(agitator),
      agitatorRotateCommand(
          agitator,
          agitatorChangeAngle,
          agitatorDesiredRotateTime,
          false,
          minAgitatorRotateTime),
      agitatorUnjamCommand(agitator, maxUnjamAngle),
      unjamSequenceCommencing(false)
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
    if (connectedAgitator->isAgitatorJammed() && !unjamSequenceCommencing)
    {
        // when the agitator is jammed, add the agitatorUnjamCommand
        // the to scheduler. The rotate forward command will be automatically
        // unscheduled.
        unjamSequenceCommencing = true;
        this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&agitatorUnjamCommand));
    }
    this->comprisedCommandScheduler.run();
}

void ShootComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&agitatorUnjamCommand),
        interrupted);
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&agitatorRotateCommand),
        interrupted);
}

bool ShootComprisedCommand::isFinished() const
{
    return (agitatorRotateCommand.isFinished() && !unjamSequenceCommencing) ||
           (agitatorUnjamCommand.isFinished() && unjamSequenceCommencing);
}

}  // namespace agitator

}  // namespace aruwsrc
