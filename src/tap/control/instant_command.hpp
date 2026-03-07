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

#ifndef TAPROOT_INSTANT_COMMAND_HPP_
#define TAPROOT_INSTANT_COMMAND_HPP_

#include <functional>
#include <array>
#include <vector>
#include <unordered_set>

#include "command.hpp"
#include "subsystem.hpp"
#include "command_scheduler.hpp"
#include "tap/drivers.hpp"

namespace tap
{
namespace control
{
template<size_t SUBSYSTEMS>
/**
 * A class for a command that runs once and then finishes. Deschedules any commands with conflicting subsystems before
 * it executes, then reschedules those descheduled commands.
 */
class InstantCommand : public Command {
public:
    InstantCommand(Drivers *drivers, std::function<void()> actionToRun, std::array<Subsystem*, SUBSYSTEMS> dependencies)
        : Command(), drivers(drivers), actionToRun(actionToRun), dependencies(dependencies) {
            for (Subsystem *dep : dependencies) subsystemRequirements |= (1UL << dep->getGlobalIdentifier());
        }

    bool isReady() override { return true; }

    void initialize() override {
        std::unordered_set<Command*> defaultCommands;
        for (Subsystem *subsystem : dependencies) {
            if (subsystem->getDefaultCommand() != nullptr) {
                defaultCommands.insert(subsystem->getDefaultCommand());
            }
        }
        
        for (Command *command : drivers->commandScheduler.getAllScheduledCommands()) {
            subsystem_scheduler_bitmap_t reqs = command->getRequirementsBitwise();
            // only add to commands to reschedule if not a default command (automatically rescheduled) and requirements overlap with instant command
            if (defaultCommands.find(command) == defaultCommands.end() && (reqs & subsystemRequirements) != 0) {
                commandsToReschedule.push_back(command);
            }
            drivers->commandScheduler.removeCommand(command, true);
        }
        actionToRun();
    }

    void execute() override {}

    void end(bool) override
    {
        // reschedule any descheduled commands
        for (Command *command : commandsToReschedule) {
            drivers->commandScheduler.addCommand(command);
        }
        commandsToReschedule.clear();
    }

    bool isFinished() const override { return true; }

    const char* getName() const override { return "instant command"; }

private:
    Drivers *drivers;
    std::function<void()> actionToRun;
    std::array<Subsystem*, SUBSYSTEMS> dependencies;

    std::vector<Command*> commandsToReschedule;
    subsystem_scheduler_bitmap_t subsystemRequirements = 0;
};
}
}

#endif