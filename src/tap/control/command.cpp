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

#include <array>
#include <utility>
#include <vector>

#include "command_scheduler.hpp"
#include "concurrent_command.hpp"
#include "conditional_command.hpp"
#include "instant_command.hpp"
#include "runnable_command.hpp"
#include "sequential_command.hpp"
#include "subsystem.hpp"
#include "switchable_command.hpp"
#include "timeout_command.hpp"

namespace tap
{
namespace control
{
namespace
{
/**
 * A race command whose second command was allocated by a composition helper, so it is
 * deallocated when the race command goes out of scope.
 */
class RaceWithOwnedCommand : public ConcurrentRaceCommand<2>
{
public:
    RaceWithOwnedCommand(Command* command, Command* ownedCommand, const char* name)
        : ConcurrentRaceCommand<2>(std::array<Command*, 2>{command, ownedCommand}, name),
          ownedCommand(ownedCommand)
    {
    }

    ~RaceWithOwnedCommand() override { delete ownedCommand; }

private:
    Command* ownedCommand;
};
}  // namespace

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

void Command::addCommand(Command*) {}

bool Command::isReady() { return true; }

Command* Command::andThen(Command* next) { return andThen({next}); }

Command* Command::andThen(std::initializer_list<Command*> next)
{
    std::vector<Command*> commands;
    commands.reserve(next.size() + 1);
    commands.push_back(this);
    commands.insert(commands.end(), next.begin(), next.end());
    return new SequentialCommand<0>(std::move(commands));
}

Command* Command::alongWith(Command* parallel)
{
    return new WeakConcurrentCommand<2>(
        std::array<Command*, 2>{this, parallel},
        "concurrent command: parallel");
}

Command* Command::alongWith(std::initializer_list<Command*> parallel)
{
    std::vector<Command*> commands;
    commands.reserve(parallel.size() + 1);
    commands.push_back(this);
    commands.insert(commands.end(), parallel.begin(), parallel.end());
    return new WeakConcurrentCommand<0>(std::move(commands), "concurrent command: parallel");
}

Command* Command::until(std::function<bool()> condition)
{
    return new RaceWithOwnedCommand(
        this,
        new ConditionalCommand([condition]() { return !condition(); }),
        "conditional race: until");
}

Command* Command::onlyWhile(std::function<bool()> condition)
{
    return new RaceWithOwnedCommand(
        this,
        new ConditionalCommand(std::move(condition)),
        "conditional race: onlyWhile");
}

Command* Command::withTimeout(uint32_t timeout)
{
    return new RaceWithOwnedCommand(
        this,
        new TimeoutCommand(timeout),
        "concurrent race: withTimeout");
}

Command* Command::deadlineWith(Command* deadline)
{
    return new ConcurrentDeadlineCommand<1>(
        std::array<Command*, 1>{this},
        "concurrent deadline",
        deadline);
}

Command* Command::sequence(std::initializer_list<Command*> next) { return andThen(next); }

Command* Command::parallel(std::initializer_list<Command*> parallel)
{
    return alongWith(parallel);
}

Command* Command::runOnce(
    std::function<void()> action,
    std::initializer_list<Subsystem*> requirements)
{
    auto* command = new InstantCommand<0>(std::move(action), {});
    for (Subsystem* requirement : requirements)
    {
        command->addSubsystemRequirement(requirement);
    }
    return command;
}

Command* Command::run(std::function<void()> action, std::initializer_list<Subsystem*> requirements)
{
    auto* command = new RunnableCommand<0>(std::move(action), {});
    for (Subsystem* requirement : requirements)
    {
        command->addSubsystemRequirement(requirement);
    }
    return command;
}

Command* Command::either(Command* onTrue, Command* onFalse, std::function<bool()> selector)
{
    return new SwitchableCommand(onTrue, onFalse, std::move(selector));
}

}  // namespace control
}  // namespace tap
