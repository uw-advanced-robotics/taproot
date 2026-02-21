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

#ifndef TAPROOT_COMMAND_COMPOSITION_HELPER_HPP_
#define TAPROOT_COMMAND_COMPOSITION_HELPER_HPP_

#include "command.hpp"
#include "concurrent_command.hpp"
#include "conditional_command.hpp"
#include "sequential_command.hpp"
#include "timeout_command.hpp"

namespace tap
{
namespace control
{
class CommandCompositionHelper
{
    /**
     * Creates a command group that runs two commands sequentially.
     * @return SequentialCommand* of the input commands.
     */
    template <size_t COMMANDS>
    SequentialCommand<COMMANDS>* sequence(Command* first, Command* second)
    {
        return new SequentialCommand({first, second});
    }

    /**
     * Creates a command group that runs two commands in parallel.
     * @return ConcurrentCommand* of the input commands.
     */
    template <size_t COMMANDS>
    ConcurrentCommand<COMMANDS>* parallel(Command* command, Command* otherCommand)
    {
        return new ConcurrentCommand({command, otherCommand}, "concurrent");
    }

    /**
     * Adds a condition to run the input command only while the input condition is true.
     * @return ConcurrentRaceCommand* of the input command and a ConditionalCommand.
     */
    template <size_t COMMANDS>
    ConcurrentRaceCommand<COMMANDS>* onlyWhile(Command* command, std::function<bool()> condition)
    {
        std::function<bool()> negated = [condition]() { return !condition(); };
        return new ConcurrentRaceCommand(
            {command, new ConditionalCommand(negated)},
            "conditional race: onlyWhile");
    }

    /**
     * Adds a condition to run the input command until the input condition becomes true.
     * @return ConcurrentRaceCommand* of the input command and a ConditionalCommand.
     */
    template <size_t COMMANDS>
    ConcurrentRaceCommand<COMMANDS>* until(Command* command, std::function<bool()> condition)
    {
        return new ConcurrentRaceCommand(
            {command, new ConditionalCommand(condition)},
            "conditional race: until");
    }

    /**
     * Adds a timeout to run the input command for a specific amount of time.
     * @return ConcurrentRaceCommand* of the input command and a TimeoutCommand.
     */
    template <size_t COMMANDS>
    ConcurrentRaceCommand<COMMANDS>* withTimeout(Command* command, uint32_t timeout)
    {
        return new ConcurrentRaceCommand(
            {command, new TimeoutCommand(timeout)},
            "concurrent race: withTimeout");
    }

    /**
     * Adds a deadline command to terminate the input command once the deadline command is
     * finished.
     * @return ConcurrentDeadlineCommand of input command deadlined with the input command.
     */
    template <size_t COMMANDS>
    ConcurrentDeadlineCommand<COMMANDS>* deadlineWith(Command* command, Command* deadlineCommand)
    {
        return new ConcurrentDeadlineCommand({command}, "concurrent deadline", deadlineCommand);
    }
};
}  // namespace control
}  // namespace tap

#endif