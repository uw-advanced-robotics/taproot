/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_CONDITIONALLY_EXECUTED_COMMAND_HPP_
#define TAPROOT_CONDITIONALLY_EXECUTED_COMMAND_HPP_

#include <cassert>
#include <list>
#include <vector>

#include "command.hpp"

namespace tap::control
{
/**
 * An interface that is used to gate the execution of a Command. Override this interface to gate
 * various commands based on some conditional logic. For example, createa a sub-class of this
 * interface and have isReady return true when the ref system indicates you have enough heat to
 * launch a projectile. Then, use a ConditionallyExecutedCommand to only run a command that launches
 * a projectile when the CommandGovernorInterface sub-object you created is true.
 */
class CommandGovernorInterface
{
public:
    virtual bool isReady() = 0;
    virtual bool isFinished() = 0;
};

template <size_t NUM_CONDITIONS>
class ConditionallyExecutedCommand : public Command
{
public:
    ConditionallyExecutedCommand(
        std::vector<Subsystem *> subRequirements,
        Command &command,
        const std::array<CommandGovernorInterface *, NUM_CONDITIONS> &commandGovernorList)
        : command(command),
          commandGovernorList(commandGovernorList)
    {
        std::for_each(subRequirements.begin(), subRequirements.end(), [&](auto sub) {
            addSubsystemRequirement(sub);
        });
        assert(command.getRequirementsBitwise() == this->getRequirementsBitwise());
    }

    const char *getName() const override { return command.getName(); }

    bool isReady() override
    {
        return std::all_of(
                   commandGovernorList.begin(),
                   commandGovernorList.end(),
                   [](auto governor) { return governor.isReady(); }) &&
               command.isReady();
    }

    void initialize() override { command.initialize(); }

    void execute() override { command.execute(); }

    void end(bool interrupted) override { command.end(interrupted); }

    bool isFinished() const override
    {
        return std::any_of(
                   commandGovernorList.begin(),
                   commandGovernorList.end(),
                   [](auto governor) { return governor.isReady(); }) ||
               command.isFinished();
    }

private:
    Command &command;

    std::array<CommandGovernorInterface *, NUM_CONDITIONS> commandGovernorList;
};
}  // namespace tap::control

#endif  // TAPROOT_CONDITIONALLY_EXECUTED_COMMAND_HPP_
