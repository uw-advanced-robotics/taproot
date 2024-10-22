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

#include <array>

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
template <size_t COMMANDS, bool RACE>
class ConcurrentTemplateCommand : public Command
{
public:
    ConcurrentTemplateCommand(std::array<Command*, COMMANDS> commands, const char* name)
        : Command(),
          commands(commands),
          name(name),
          finishedCommands(0),
          allCommands(0)
    {
        for (Command* command : commands)
        {
            modm_assert(
                command != nullptr,
                "ConcurrentCommand::ConcurrentCommand",
                "Null pointer command passed into concurrent command.");
            auto requirements = command->getRequirementsBitwise();
            modm_assert(
                (this->commandRequirementsBitwise & requirements) == 0,
                "ConcurrentCommand::ConcurrentCommand",
                "Multiple commands to concurrent command have overlapping requirements.");
            this->commandRequirementsBitwise |= requirements;
            this->allCommands |= (1ull << command->getGlobalIdentifier());
        }
    }

    const char* getName() const override { return this->name; }

    bool isReady() override
    {
        for (Command* command : commands)
        {
            if (!command->isReady())
            {
                return false;
            }
        }
        return true;
    }

    void initialize() override
    {
        for (Command* command : commands)
        {
            command->initialize();
        }
    }

    void execute() override
    {
        for (Command* command : commands)
        {
            if (!(this->finishedCommands & (1ull << command->getGlobalIdentifier())))
            {
                command->execute();
                if (command->isFinished())
                {
                    command->end(false);
                    this->finishedCommands |= 1ull << command->getGlobalIdentifier();
                }
            }
        }
    }

    void end(bool interrupted) override
    {
        for (Command* command : commands)
        {
            if (!(this->finishedCommands & (1ull << command->getGlobalIdentifier())))
            {
                if (RACE)
                {
                    command->end(true);
                }
                else
                {
                    command->end(interrupted);
                }
            }
        }
    }

    bool isFinished() const override
    {
        if (RACE)
        {
            return this->finishedCommands != 0;
        }
        return this->finishedCommands == this->allCommands;
    }

private:
    std::array<Command*, COMMANDS> commands;
    const char* name;
    command_scheduler_bitmap_t finishedCommands;
    command_scheduler_bitmap_t allCommands;
};  // class ConcurrentTemplateCommand

/**
 * Runs commands in parallel until all are finished.
 */
template <size_t COMMANDS>
using ConcurrentCommand = ConcurrentTemplateCommand<COMMANDS, false>;

/**
 * Runs commands in parallel until one is finished.
 */
template <size_t COMMANDS>
using ConcurrentRaceCommand = ConcurrentTemplateCommand<COMMANDS, true>;

}  // namespace control

}  // namespace tap

#endif  // TAPROOT_CONCURRENT_COMMAND_HPP_
