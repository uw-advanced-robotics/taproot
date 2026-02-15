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

#include "command.hpp"

#include "command_scheduler.hpp"
#include "conditional_command.hpp"
#include "subsystem.hpp"
#include "timeout_command.hpp"

namespace tap
{
namespace control
{
Command::Command() : globalIdentifier(CommandScheduler::constructCommand(this)) {}

Command::~Command() { CommandScheduler::destructCommand(this); }

void Command::addSubsystemRequirement(Subsystem* requirement)
{
    if (requirement == nullptr)
    {
        return;
    }
    commandRequirementsBitwise |= (1UL << requirement->getGlobalIdentifier());
}

bool Command::isReady() { return true; }

SequentialCommand* Command::andThen(Command* command) &&
{
    return new SequentialCommand({this, command});
}

SequentialCommand* Command::beforeStarting(Command* command) &&
{
    return new SequentialCommand({command, this});
}

ConcurrentCommand* Command::alongWith(Command* command) &&
{
    return new ConcurrentCommand({this, command}, "concurrent");
}

ConcurrentRaceCommand* Command::onlyWhile(std::function<bool()> condition) &&
{
    std::function<bool()> negated = [condition]() { return !condition(); };
    return new ConcurrentRaceCommand(
        {this, new ConditionalCommand(negated)},
        "conditional race: onlyWhile");
}

ConcurrentRaceCommand* Command::until(std::function<bool()> condition) &&
{
    return new ConcurrentRaceCommand(
        {this, new ConditionalCommand(condition)},
        "conditional race: until");
}

ConcurrentRaceCommand* Command::withTimeout(uint32_t timeout) &&
{
    return new ConcurrentRaceCommand(
        {this, new TimeoutCommand(timeout)},
        "concurrent race: withTimeout");
}

ConcurrentDeadlineCommand* Command::deadlineWith(Command* command) &&
{
    return new ConcurrentDeadlineCommand({this}, "concurrent deadline", command);
}

}  // namespace control
}  // namespace tap
