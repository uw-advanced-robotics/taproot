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

#ifndef TAPROOT_ALTERNATE_COMMAND_HPP_
#define TAPROOT_ALTERNATE_COMMAND_HPP_

#include <algorithm>
#include <array>
#include <cassert>
#include <cinttypes>
#include <vector>

#include "command.hpp"
#include "command_governor_interface.hpp"

namespace tap::control
{
/**
 * A command that alternates between two commands based on the state of a list of governors. One of
 * the two commands is selected in the `isReady` phase of the Command, at which point that command
 * will be executed until its completion.
 *
 * @tparam NUM_CONDITIONS The number of governors in the governor list.
 */
template <std::size_t NUM_CONDITIONS>
class AlternateCommand : public Command
{
public:
    AlternateCommand(
        std::vector<Subsystem *> subRequirements,
        Command &commandWhenGovernorsTrue,
        Command &commandWhenGovernorsFalse,
        const std::array<CommandGovernorInterface *, NUM_CONDITIONS> &commandGovernorList)
        : commandWhenGovernorsTrue(commandWhenGovernorsTrue),
          commandWhenGovernorsFalse(commandWhenGovernorsFalse),
          commandGovernorList(commandGovernorList)
    {
        std::for_each(
            subRequirements.begin(),
            subRequirements.end(),
            [&](auto sub) { addSubsystemRequirement(sub); });

        assert(commandWhenGovernorsTrue.getRequirementsBitwise() == this->getRequirementsBitwise());
        assert(
            commandWhenGovernorsFalse.getRequirementsBitwise() == this->getRequirementsBitwise());
    }

    const char *getName() const override { return "Alternate command"; }

    bool isReady() override
    {
        readyWhenGovernorsTrue = std::all_of(
            commandGovernorList.begin(),
            commandGovernorList.end(),
            [](auto governor) { return governor->isReady(); });

        return (readyWhenGovernorsTrue && commandWhenGovernorsTrue.isReady()) ||
               (!readyWhenGovernorsTrue && commandWhenGovernorsFalse.isReady());
    }

    void initialize() override
    {
        if (readyWhenGovernorsTrue)
        {
            commandWhenGovernorsTrue.initialize();
        }
        else
        {
            commandWhenGovernorsFalse.initialize();
        }
    }

    void execute() override
    {
        if (readyWhenGovernorsTrue)
        {
            commandWhenGovernorsTrue.execute();
        }
        else
        {
            commandWhenGovernorsFalse.execute();
        }
    }

    void end(bool interrupted) override
    {
        if (readyWhenGovernorsTrue)
        {
            commandWhenGovernorsTrue.end(interrupted);
        }
        else
        {
            commandWhenGovernorsFalse.end(interrupted);
        }
    }

    bool isFinished() const override
    {
        return readyWhenGovernorsTrue ? commandWhenGovernorsTrue.isFinished()
                                      : commandWhenGovernorsFalse.isFinished();
    }

private:
    bool readyWhenGovernorsTrue = false;
    Command &commandWhenGovernorsTrue;
    Command &commandWhenGovernorsFalse;

    std::array<CommandGovernorInterface *, NUM_CONDITIONS> commandGovernorList;
};
}  // namespace tap::control

#endif  // TAPROOT_ALTERNATE_COMMAND_HPP_