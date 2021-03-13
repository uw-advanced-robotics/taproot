/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "command_scheduler.hpp"

#include <algorithm>
#include <set>
#include <utility>

#include "aruwlib/Drivers.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/errors/create_errors.hpp"

using namespace std;

namespace aruwlib
{
namespace control
{
uint32_t CommandScheduler::commandSchedulerTimestamp = 0;

void CommandScheduler::addCommand(Command* commandToAdd)
{
    if (this->runningHardwareTests)
    {
        RAISE_ERROR(
            drivers,
            "attempting to add command while running tests",
            aruwlib::errors::Location::COMMAND_SCHEDULER,
            aruwlib::errors::CommandSchedulerErrorType::ADDING_NULLPTR_COMMAND);
        return;
    }

    if (commandToAdd == nullptr)
    {
        RAISE_ERROR(
            drivers,
            "attempting to add nullptr command",
            aruwlib::errors::Location::COMMAND_SCHEDULER,
            aruwlib::errors::CommandSchedulerErrorType::ADDING_NULLPTR_COMMAND);
        return;
    }

    bool commandAdded = false;

    const set<Subsystem*>& commandRequirements = commandToAdd->getRequirements();

    // Check to see if all the requirements are in the subsytemToCommandMap
    for (auto& requirement : commandRequirements)
    {
        if (subsystemToCommandMap.find(requirement) == subsystemToCommandMap.end())
        {
            // the command you are trying to add has a subsystem that is not in the
            // scheduler, so you cannot add it (will lead to undefined control behavior)
            RAISE_ERROR(
                drivers,
                "Attempting to add a command without subsystem in the scheduler",
                aruwlib::errors::Location::COMMAND_SCHEDULER,
                aruwlib::errors::CommandSchedulerErrorType::RUN_TIME_OVERFLOW);
            return;
        }
    }

    // end all commands running on the subsystem requirements.
    // They were interrupted.
    // Additionally, replace the current command with the commandToAdd
    for (auto& requirement : commandRequirements)
    {
        map<Subsystem*, Command*>::iterator subsystemRequirementCommandPair =
            subsystemToCommandMap.find(requirement);

        if (subsystemRequirementCommandPair->second != nullptr)
        {
            Command* commandToCancel = subsystemRequirementCommandPair->second;
            // end the command
            commandToCancel->end(true);
            // Loop through all the subsystems, and if they use this command,
            // remove it to make sure it doesnt continue running.
            // This also prevents end from being called twice on the same command.
            for (auto& subsystemToCommandEntry : subsystemToCommandMap)
            {
                if (subsystemToCommandEntry.second == commandToCancel)
                {
                    subsystemToCommandEntry.second = nullptr;
                }
            }
        }
        subsystemRequirementCommandPair->second = commandToAdd;
        commandAdded = true;
    }

    // initialize the commandToAdd. Only do this once even though potentially
    // multiple subsystems rely on this command.
    if (commandAdded)
    {
        commandToAdd->initialize();
    }
}

void CommandScheduler::run()
{
    uint32_t runStart = aruwlib::arch::clock::getTimeMicroseconds();
    // Timestamp for reference and for disallowing a command from running
    // multiple times during the same call to run.
    if (this == &drivers->commandScheduler)
    {
        commandSchedulerTimestamp++;
    }

    if (this->runningHardwareTests)
    {
        for (auto& subsystemToCommand : subsystemToCommandMap)
        {
            if (!subsystemToCommand.first->isHardwareTestComplete())
            {
                subsystemToCommand.first->runHardwareTests();
            }
        }
        return;
    }

    // refresh all and run all commands
    for (auto& currSubsystemCommandPair : subsystemToCommandMap)
    {
        // add default command if no command is currently being run
        if (currSubsystemCommandPair.second == nullptr &&
            currSubsystemCommandPair.first->getDefaultCommand() != nullptr)
        {
            addCommand(currSubsystemCommandPair.first->getDefaultCommand());
        }
        // only run the command if it hasn't been run this time run has been called
        if (currSubsystemCommandPair.second != nullptr)
        {
            Command* currCommand = currSubsystemCommandPair.second;

            if (currCommand->prevSchedulerExecuteTimestamp != commandSchedulerTimestamp)
            {
                currCommand->execute();
                currCommand->prevSchedulerExecuteTimestamp = commandSchedulerTimestamp;
            }
            // remove command if finished running
            if (currCommand->isFinished())
            {
                currCommand->end(false);
                currSubsystemCommandPair.second = nullptr;
            }
        }
        // refresh subsystem
        if (currSubsystemCommandPair.first->prevSchedulerExecuteTimestamp !=
            commandSchedulerTimestamp)
        {
            currSubsystemCommandPair.first->refresh();
            currSubsystemCommandPair.first->prevSchedulerExecuteTimestamp =
                commandSchedulerTimestamp;
        }
    }
    // make sure we are not going over tolerable runtime, otherwise something is really
    // wrong with the code
    uint32_t runEnd = aruwlib::arch::clock::getTimeMicroseconds();
    if (runEnd - runStart > MAX_ALLOWABLE_SCHEDULER_RUNTIME)
    {
        // shouldn't take more than 1 ms to complete all this stuff, if it does something
        // is seriously wrong (i.e. you are adding subsystems unchecked)
        RAISE_ERROR(
            drivers,
            "scheduler took longer than MAX_ALLOWABLE_SCHEDULER_RUNTIME",
            aruwlib::errors::Location::COMMAND_SCHEDULER,
            aruwlib::errors::CommandSchedulerErrorType::RUN_TIME_OVERFLOW);
    }
}

void CommandScheduler::removeCommand(Command* command, bool interrupted)
{
    if (command == nullptr)
    {
        return;
    }
    bool commandFound = false;
    for (auto& subsystemCommandPair : subsystemToCommandMap)
    {
        if (subsystemCommandPair.second == command)
        {
            if (!commandFound)
            {
                subsystemCommandPair.second->end(interrupted);
                commandFound = true;
            }
            subsystemCommandPair.second = nullptr;
        }
    }
}

bool CommandScheduler::isCommandScheduled(Command* command) const
{
    if (command == nullptr)
    {
        return false;
    }
    return std::any_of(
        subsystemToCommandMap.begin(),
        subsystemToCommandMap.end(),
        [command](pair<Subsystem*, Command*> p) { return p.second == command; });
}

void CommandScheduler::registerSubsystem(Subsystem* subsystem)
{
    if (subsystem != nullptr && !isSubsystemRegistered(subsystem))
    {
        subsystemToCommandMap[subsystem] = nullptr;
    }
    else
    {
        RAISE_ERROR(
            drivers,
            "subsystem is already added or trying to add nullptr subsystem",
            aruwlib::errors::Location::COMMAND_SCHEDULER,
            aruwlib::errors::CommandSchedulerErrorType::ADDING_NULLPTR_COMMAND);
    }
}

bool CommandScheduler::isSubsystemRegistered(Subsystem* subsystem) const
{
    if (subsystem == nullptr)
    {
        return false;
    }
    return subsystemToCommandMap.find(subsystem) != subsystemToCommandMap.end();
}

void CommandScheduler::runSubsystemTests()
{
    this->runningHardwareTests = true;
    for (auto& pair : this->subsystemToCommandMap)
    {
        pair.first->hardwareTestsComplete = false;
        if (pair.second != nullptr)
        {
            pair.second->end(true);
            pair.second = nullptr;
        }
    }
}

void CommandScheduler::stopHardwareTests()
{
    this->runningHardwareTests = false;
    for (auto& pair : this->subsystemToCommandMap)
    {
        if (pair.first->isHardwareTestComplete())
        {
            pair.first->setHardwareTestsComplete();
        }
    }
}
}  // namespace control

}  // namespace aruwlib
