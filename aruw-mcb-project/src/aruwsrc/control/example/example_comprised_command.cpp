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

#include "example_comprised_command.hpp"

#include "example_command.hpp"
#include "example_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
ExampleComprisedCommand::ExampleComprisedCommand(
    aruwlib::Drivers* drivers,
    ExampleSubsystem* subsystem)
    : aruwlib::control::ComprisedCommand(drivers),
      exampleCommand(subsystem, 2000),
      otherExampleCommand(subsystem, 500),
      switchTimer(2000),
      switchCommand(false)
{
    this->addSubsystemRequirement(subsystem);
    this->comprisedCommandScheduler.registerSubsystem(subsystem);
}

void ExampleComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&exampleCommand));
}

void ExampleComprisedCommand::execute()
{
    if (switchTimer.execute())
    {
        switchTimer.restart(2000);
        if (switchCommand)
        {
            comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&otherExampleCommand));
        }
        else
        {
            comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&exampleCommand));
        }
        switchCommand = !switchCommand;
    }

    this->comprisedCommandScheduler.run();
}

void ExampleComprisedCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(dynamic_cast<Command*>(&exampleCommand), interrupted);
}

}  // namespace control

}  // namespace aruwsrc
