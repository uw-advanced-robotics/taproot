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

#ifndef COMMAND_HPP_
#define COMMAND_HPP_

#include <set>

#include "mock_macros.hpp"

namespace aruwlib
{
namespace control
{
class Subsystem;

/**
 * A generic extendable class for implementing a command. Each
 * command is attached to a subsystem. To create a new command,
 * extend the Command class and instantiate the virtual functions
 * in this class. See example_command.hpp for example of this.
 */
class Command
{
public:
    Command() : prevSchedulerExecuteTimestamp(0) {}

    /**
     * Specifies the set of subsystems used by this command. Two commands cannot
     * use the same subsystem at the same time.  If another command is scheduled
     * that shares a requirement, the command will be interrupted. If no subsystems
     * are required, return an empty set.
     *
     * The generic Command class contains a list of the requrements. The user
     * should add requirements to this list accordingly (typically in the constructor
     * of a class extending the Command class). If a Command does not specify any
     * requirements, the Command cannot be added to the CommandScheduler.
     *
     * @see CommandScheduler
     * @return the set of subsystems that are required.
     */
    mockable const std::set<Subsystem*>& getRequirements() const;

    /**
     * Returns whether the command requires a given subsystem.
     *
     * @param[in] requirement the subsystem to inquire about.
     * @return `true` if the subsystem is required for this Command, `false` otherwise.
     */
    mockable bool hasRequirement(Subsystem* requirement) const;

    /**
     * Adds the required subsystem to a list of required subsystems.
     *
     * @param[in] requirement the requirement to add to the list of requirements.
     *      If the requirement is nullptr or if the requirement is already in the
     *      set, nothing is added.
     */
    mockable void addSubsystemRequirement(Subsystem* requirement);

    /**
     * @return the name of the command, to be implemented by derived classes.
     */
    virtual const char* getName() const = 0;

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled by a CommandScheduler.
     */
    virtual void initialize() = 0;

    /**
     * The main body of a command. Called repeatedly while the command is
     * scheduled by a CommandScheduler.
     */
    virtual void execute() = 0;

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally, or when it interrupted/canceled.
     *
     * @param[in] interrupted whether the command was interrupted/canceled.
     */
    virtual void end(bool interrupted) = 0;

    /**
     * Whether the command has finished. Once a Command finishes, the scheduler
     * will call the `end()` function and un-schedule it. If a Command is naturally
     * finished (i.e. `isFinished() == true`), then the CommandScheduler will pass
     * in `false` to `end()`. If, for example, another Command is added that in turn
     * stops the Command from executing, then the CommandScheduler will pass in `true`
     * to `end()`.
     *
     * @return whether the command has finished.
     */
    virtual bool isFinished() const = 0;

private:
    friend class CommandScheduler;

    uint32_t prevSchedulerExecuteTimestamp;

    // contains pointers to const Subsystem pointers that this command requires
    std::set<Subsystem*> commandRequirements;
};  // class Command

}  // namespace control

}  // namespace aruwlib

#endif  // COMMAND_HPP_
