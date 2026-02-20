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

#ifndef TAPROOT_CONCURRENT_COMMAND_HPP_
#define TAPROOT_CONCURRENT_COMMAND_HPP_

#include <vector>

#include "modm/architecture/interface/assert.hpp"

#include "command.hpp"
#include "command_scheduler_types.hpp"

namespace tap
{
namespace control
{
/**
 * A command that runs multiple commands in parallel. Waits for all passed in commands to be ready
 * before being ready itself. When RACE is false, it continues executing until all passed in
 * commands have finished and then the concurrent command finishes. When RACE is true, only one
 * passed in command needs to finish for the concurrent command to finish.
 */
template <bool RACE>
class ConcurrentTemplateCommand : public Command
{
public:
    ConcurrentTemplateCommand(
        std::vector<Command*> commands,
        const char* name,
        Command* deadlineCommand = nullptr);

    const char* getName() const override;
    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;
    void addCommand(Command* command) override;

private:
    std::vector<Command*> commands;
    Command* deadlineCommand;
    const char* name;
    command_scheduler_bitmap_t finishedCommands;
    command_scheduler_bitmap_t allCommands;
};

using ConcurrentCommand = ConcurrentTemplateCommand<false>;
using ConcurrentRaceCommand = ConcurrentTemplateCommand<true>;
using ConcurrentDeadlineCommand = ConcurrentTemplateCommand<false>;

template <bool RACE>
ConcurrentTemplateCommand<RACE>::ConcurrentTemplateCommand(
    std::vector<Command*> commands,
    const char* name,
    Command* deadlineCommand)
    : Command(),
      commands(commands),
      deadlineCommand(deadlineCommand),
      name(name),
      finishedCommands(0),
      allCommands(0)
{
    for (Command* command : commands)
    {
        modm_assert(command != nullptr, "ConcurrentCommand", "Null command passed");
        auto requirements = command->getRequirementsBitwise();
        modm_assert(
            (commandRequirementsBitwise & requirements) == 0,
            "ConcurrentCommand",
            "Overlapping requirements");
        commandRequirementsBitwise |= requirements;
        allCommands |= 1ull << command->getGlobalIdentifier();
    }
}

template <bool RACE>
const char* ConcurrentTemplateCommand<RACE>::getName() const
{
    return name;
}

template <bool RACE>
bool ConcurrentTemplateCommand<RACE>::isReady()
{
    for (Command* command : commands)
        if (!command->isReady()) return false;
    return true;
}

template <bool RACE>
void ConcurrentTemplateCommand<RACE>::initialize()
{
    for (Command* command : commands) command->initialize();
}

template <bool RACE>
void ConcurrentTemplateCommand<RACE>::execute()
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

template <bool RACE>
void ConcurrentTemplateCommand<RACE>::end(bool interrupted)
{
    for (Command* command : commands)
    {
        if (!(finishedCommands & (1ull << command->getGlobalIdentifier())))
        {
            if (RACE)
                command->end(true);
            else
                command->end(interrupted);
        }
    }
}

template <bool RACE>
bool ConcurrentTemplateCommand<RACE>::isFinished() const
{
    if (deadlineCommand != nullptr &&
        (finishedCommands & (1ull << deadlineCommand->getGlobalIdentifier())))
        return true;

    if (RACE) return finishedCommands != 0;

    return finishedCommands == allCommands;
}

template <bool RACE>
void ConcurrentTemplateCommand<RACE>::addCommand(Command* command)
{
    commands.push_back(command);
}

}  // namespace control
}  // namespace tap

#endif