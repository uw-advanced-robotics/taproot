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

#ifndef TAPROOT_RUNNABLE_COMMAND_HPP_
#define TAPROOT_RUNNABLE_COMMAND_HPP_

#include <array>
#include <functional>

#include "tap/drivers.hpp"

#include "command.hpp"
#include "repeat_command.hpp"
#include "subsystem.hpp"

namespace tap
{
namespace control
{
template <size_t SUBSYSTEMS>
/**
 * A class for a command that runs a given action every loop of the command scheduler until
 * interrupted.
 */
class RunnableCommand : public Command
{
public:
    RunnableCommand(
        std::function<void()> actionToRun,
        std::array<Subsystem *, SUBSYSTEMS> dependencies)
        : Command(),
          actionToRun(actionToRun)
    {
        for (Subsystem *dep : dependencies) addSubsystemRequirement(dep);
    }

    bool isReady() override { return true; }

    void initialize() override {}

    void execute() override { actionToRun(); }

    void end(bool) override {}

    bool isFinished() const override { return false; }

    const char *getName() const override { return "runnable command"; }

private:
    std::function<void()> actionToRun;
};
}  // namespace control
}  // namespace tap

#endif