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
#include <vector>

#include "command.hpp"

namespace tap::control
{

class ConditionallyExecutedCommand : public Command
{
public:
    using ConditionalFn = bool (*)();

    ConditionallyExecutedCommand(
        std::vector<Subsystem *> subRequirements,
        Command &command,
        ConditionalFn fn)
        : command(command),
          fn(fn)
    {
        std::for_each(
            subRequirements.begin(),
            subRequirements.end(),
            [&](auto sub) { addSubsystemRequirement(sub); });
        assert(command.getRequirementsBitwise() == this->getRequirementsBitwise());
        assert(fn != nullptr);
    }

    const char *getName() const override { return command.getName(); }
    bool isReady() override { return fn() && command.isReady(); }
    void initialize() override { command.initialize(); }
    void execute() override { command.execute(); }
    void end(bool interrupted) override { command.end(interrupted); }
    bool isFinished() const override { return fn() && command.isFinished(); }

private:
    Command &command;
    ConditionalFn fn;
};
}  // namespace tap::control

#endif  // TAPROOT_CONDITIONALLY_EXECUTED_COMMAND_HPP_
