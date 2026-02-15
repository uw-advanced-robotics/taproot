/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sequential_command.hpp"

#include "command.hpp"

namespace tap
{
namespace control
{
SequentialCommand::SequentialCommand(const std::vector<Command*>& commands)
    : commands(commands),
      finishedCommands(0),
      allCommands(0)
{
    for (Command* command : commands)
    {
        auto req = command->getRequirementsBitwise();
        allCommands |= 1ull << command->getGlobalIdentifier();
        commandRequirementsBitwise |= req;
    }
}

const char* SequentialCommand::getName() const { return name; }

bool SequentialCommand::isReady()
{
    for (Command* command : commands)
        if (!command->isReady()) return false;
    return true;
}

void SequentialCommand::initialize()
{
    for (Command* command : commands) command->initialize();
}

void SequentialCommand::execute()
{
    for (Command* command : commands)
    {
        if (!(finishedCommands & (1ull << command->getGlobalIdentifier())))
        {
            command->execute();
            if (command->isFinished())
            {
                command->end(false);
                finishedCommands |= 1ull << command->getGlobalIdentifier();
            }
        }
    }
}

void SequentialCommand::end(bool interrupted)
{
    for (Command* command : commands)
    {
        if (!(finishedCommands & (1ull << command->getGlobalIdentifier())))
        {
            command->end(interrupted);
        }
    }
}

bool SequentialCommand::isFinished() const { return finishedCommands == allCommands; }

}  // namespace control
}  // namespace tap
