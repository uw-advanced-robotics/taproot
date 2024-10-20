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

#ifndef TAPROOT_CONCURRENT_COMMAND_HPP_
#define TAPROOT_CONCURRENT_COMMAND_HPP_

#include <array>

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"
#include "tap/util_macros.hpp"

#include "command.hpp"
#include "command_scheduler.hpp"
#include "command_scheduler_types.hpp"

namespace tap
{
namespace control
{
class Subsystem;

/**
 * A generic extendable class for implementing a command. Each
 * command is attached to a subsystem. To create a new command,
 * extend the Command class and instantiate the virtual functions
 * in this class. See example_command.hpp for example of this.
 */
template <size_t COMMANDS>
class ConcurrentCommand : public Command
{
public:
    ConcurrentCommand(std::array<Command*, COMMANDS> commands, const char* name, Drivers* drivers)
        : Command(),
          commands(commands),
          name(name),
          finishedCommands(0)
    {
        for (Command* command : commands)
        {
            if (command == nullptr)
            {
                RAISE_ERROR(drivers, "Null pointer command passed into concurrent command.");
                continue;
            }
            this->commandRequirementsBitwise |= (command->getRequirementsBitwise());
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
                command->end(interrupted);
            }
        }
    }

    bool isFinished() const override
    {
        for (Command* command : commands)
        {
            if (!command->isFinished())
            {
                return false;
            }
        }
        return true;
    }

private:
    std::array<Command*, COMMANDS> commands;
    const char* name;
    command_scheduler_bitmap_t finishedCommands;
};  // class ConcurrentCommand

}  // namespace control

}  // namespace tap

#endif  // TAPROOT_CONCURRENT_COMMAND_HPP_
