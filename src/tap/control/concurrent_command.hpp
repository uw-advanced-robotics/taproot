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
#include <utility>

#include "modm/architecture/interface/assert.hpp"

#include "command.hpp"
#include "command_scheduler_types.hpp"

namespace tap
{
namespace control
{
/**
 * Requires all commands to be ready.
 * For concurrent commands scheduled directly by the command scheduler.
 */
struct StrictReadinessCheck
{
    template <size_t N>
    static bool isReady(std::array<std::pair<Command*, bool>, N>& commandsInfo)
    {
        for (const auto& info : commandsInfo)
        {
            if (!info.first->isReady())
            {
                return false;
            }
        }
        return true;
    }
};

/**
 * For scheduling concurrent commands that are owned by Trigger actions (e.g., Trigger::whileTrue)
 * in order for the command readiness to not block Trigger activation. Child commands are
 * responsible for enforcing their own readiness.
 */
struct WeakReadinessCheck
{
    template <size_t N>
    static bool isReady(std::array<std::pair<Command*, bool>, N>&)
    {
        return true;
    }
};

/**
 * A command that runs multiple commands in parallel. When RACE is false, it continues executing
 * until all passed in commands have finished and then the concurrent command finishes. When RACE is
 * true, only one passed in command needs to finish for the concurrent command to finish. If
 * `deadlineCommand` is not a null pointer, the command group runs until `deadlineCommand` is
 * finished. If any of the commands passed into commandsInfo is a repeat command, the command group
 * will never terminate unless a deadline command is provided.
 */
template <size_t COMMANDS, bool RACE, typename ReadinessCheck>
class ConcurrentTemplateCommand : public Command
{
public:
    ConcurrentTemplateCommand(
        std::array<std::pair<Command*, bool>, COMMANDS> commandsInfo,
        const char* name,
        Command* deadlineCommand = nullptr)
        : Command(),
          commandsInfo(commandsInfo),
          deadlineCommand(deadlineCommand),
          name(name),
          finishedCommands(0),
          allCommands(0)
    {
        innerCommandsEnded.fill(false);

        for (size_t i = 0; i < COMMANDS; i++)
        {
            Command* command = commandsInfo[i].first;
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
        if (deadlineCommand)
        {
            auto requirements = deadlineCommand->getRequirementsBitwise();
            modm_assert(
                (this->commandRequirementsBitwise & requirements) == 0,
                "ConcurrentCommand::ConcurrentCommand",
                "Deadline command has overlapping requirements.");
            this->commandRequirementsBitwise |= requirements;
            this->allCommands |= (1ull << deadlineCommand->getGlobalIdentifier());
        }
    }

    const char* getName() const override { return this->name; }

    bool isReady() override
    {
        if (deadlineCommand && !deadlineCommand->isReady()) return false;
        return ReadinessCheck::isReady(commandsInfo);
    }

    void initialize() override
    {
        finishedCommands = 0;
        // reset all ended states
        innerCommandsEnded.fill(false);
        for (const auto& info : commandsInfo)
        {
            info.first->initialize();
        }
        if (deadlineCommand)
        {
            deadlineCommand->initialize();
        }
    }

    void handleCommandExecution(Command* command, bool isRepeatCommand, size_t index)
    {
        if (!(this->finishedCommands & (1ull << command->getGlobalIdentifier())))
        {
            if (isRepeatCommand)
            {
                // if command has ended, reschedule it if possible
                if (innerCommandsEnded[index])
                {
                    if (!command->isReady()) return;
                    command->end(false);
                    command->initialize();
                    innerCommandsEnded[index] = false;
                }

                command->execute();
                if (command->isFinished())
                {
                    innerCommandsEnded[index] = true;
                }
            }
            else
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

    void execute() override
    {
        for (size_t i = 0; i < COMMANDS; i++)
        {
            handleCommandExecution(commandsInfo[i].first, commandsInfo[i].second, i);
        }
        if (deadlineCommand) handleCommandExecution(deadlineCommand, false, 0);
    }

    void end(bool interrupted) override
    {
        bool deadlineEnded = deadlineCommand &&
                             (finishedCommands & (1ull << deadlineCommand->getGlobalIdentifier()));
        for (size_t i = 0; i < COMMANDS; i++)
        {
            Command* command = commandsInfo[i].first;
            bool isRepeat = commandsInfo[i].second;
            bool alreadyFinished =
                this->finishedCommands & (1ull << command->getGlobalIdentifier());

            // end() already called for repeat commands in the temporary ended state unless deadline
            // command is finished (ends concurrent command)
            bool repeatWaiting = !deadlineEnded && isRepeat && innerCommandsEnded[i];
            if (!alreadyFinished && !repeatWaiting)
            {
                command->end(RACE ? true : interrupted);
            }
        }
        if (deadlineCommand && deadlineEnded) deadlineCommand->end(interrupted);
    }

    bool isFinished() const override
    {
        if (deadlineCommand &&
            (finishedCommands & (1ull << deadlineCommand->getGlobalIdentifier())))
            return true;
        if (RACE)
        {
            return this->finishedCommands != 0;
        }
        return this->finishedCommands == this->allCommands;
    }

private:
    std::array<std::pair<Command*, bool>, COMMANDS> commandsInfo;
    std::array<bool, COMMANDS>
        innerCommandsEnded;  // for commandsInfo, true if repeat command is in ended state
    Command* deadlineCommand;
    const char* name;
    command_scheduler_bitmap_t finishedCommands;
    command_scheduler_bitmap_t allCommands;
};  // class ConcurrentTemplateCommand

template <size_t COMMANDS>
using ConcurrentCommand = ConcurrentTemplateCommand<COMMANDS, false, StrictReadinessCheck>;

template <size_t COMMANDS>
using WeakConcurrentCommand = ConcurrentTemplateCommand<COMMANDS, false, WeakReadinessCheck>;

template <size_t COMMANDS>
using ConcurrentRaceCommand = ConcurrentTemplateCommand<COMMANDS, true, StrictReadinessCheck>;

template <size_t COMMANDS>
using ConcurrentDeadlineCommand = ConcurrentTemplateCommand<COMMANDS, false, StrictReadinessCheck>;

}  // namespace control
}  // namespace tap

#endif  // TAPROOT_CONCURRENT_COMMAND_HPP_