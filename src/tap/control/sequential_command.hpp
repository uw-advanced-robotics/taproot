/*
 * Copyright (c) 2024-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_SEQUENTIAL_COMMAND_HPP_
#define TAPROOT_SEQUENTIAL_COMMAND_HPP_

#include <array>
#include <type_traits>
#include <utility>
#include <vector>

#include "modm/architecture/interface/assert.hpp"

#include "command.hpp"
#include "command_scheduler_types.hpp"

namespace tap
{
namespace control
{
/**
 * Runs the given commands one after another. The sequence finishes when every command has
 * finished. Each command is initialized when it starts, not when the sequence is scheduled.
 *
 * `COMMANDS == 0` stores commands in a `std::vector` so the length can be chosen at runtime.
 * Any other size stores them in a `std::array`.
 */
template <size_t COMMANDS = 0>
class SequentialCommand : public Command
{
public:
    template <typename... Args>
    SequentialCommand(Args*... args)
        : Command(),
          commands(),
          currentCommand(0),
          commandInitialized(false)
    {
        static_assert(
            COMMANDS == 0 || sizeof...(Args) == COMMANDS,
            "SequentialCommand Error: The number of commands passed does not match the template "
            "size!");
        if constexpr (COMMANDS == 0)
        {
            commands.assign({static_cast<Command*>(args)...});
        }
        else if constexpr (sizeof...(Args) == COMMANDS)
        {
            commands = std::array<Command*, COMMANDS>{static_cast<Command*>(args)...};
        }
        registerRequirements();
    }

    /**
     * Runtime-sized sequence. Only `SequentialCommand<0>` accepts this constructor; composition
     * helpers use it when the number of commands is not a compile-time constant.
     */
    SequentialCommand(std::vector<Command*> commandsIn)
        : Command(),
          commands(),
          currentCommand(0),
          commandInitialized(false)
    {
        static_assert(
            COMMANDS == 0,
            "SequentialCommand Error: the vector constructor requires SequentialCommand<0>.");
        if constexpr (COMMANDS == 0)
        {
            commands = std::move(commandsIn);
            registerRequirements();
        }
    }

    const char* getName() const override
    {
        return this->currentCommand == commands.size()
                   ? ""
                   : this->commands[this->currentCommand]->getName();
    }

    bool isReady() override { return this->commands[0]->isReady(); }

    void initialize() override
    {
        this->currentCommand = 0;
        this->commandInitialized = false;
    }

    void execute() override
    {
        if (this->currentCommand >= commands.size()) return;
        if (!this->commandInitialized)
        {
            if (this->commands[this->currentCommand]->isReady())
            {
                this->commands[this->currentCommand]->initialize();
                this->commandInitialized = true;
            }
        }

        if (this->commandInitialized)
        {
            this->commands[this->currentCommand]->execute();

            if (this->commands[this->currentCommand]->isFinished())
            {
                this->commands[this->currentCommand]->end(false);
                this->commandInitialized = false;
                this->currentCommand++;
            }
        }
    }

    void end(bool interrupted) override
    {
        if (this->commandInitialized && this->currentCommand != commands.size())
        {
            this->commands[this->currentCommand]->end(interrupted);
        }
    }

    bool isFinished() const override { return this->currentCommand == commands.size(); }

private:
    void registerRequirements()
    {
        modm_assert(
            !commands.empty(),
            "SequentialCommand::SequentialCommand",
            "No commands passed into sequential command.");
        for (Command* command : commands)
        {
            modm_assert(
                command != nullptr,
                "SequentialCommand::SequentialCommand",
                "Null pointer command passed into sequential command.");
            this->commandRequirementsBitwise |= command->getRequirementsBitwise();
        }
    }

    using CommandStorage =
        std::conditional_t<COMMANDS == 0, std::vector<Command*>, std::array<Command*, COMMANDS>>;

    CommandStorage commands;
    size_t currentCommand;
    bool commandInitialized;
};  // class SequentialCommand

template <typename... Args>
SequentialCommand(Args*...) -> SequentialCommand<sizeof...(Args)>;

}  // namespace control

}  // namespace tap

#endif  // TAPROOT_SEQUENTIAL_COMMAND_HPP_