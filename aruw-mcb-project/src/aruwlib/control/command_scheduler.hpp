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

#ifndef COMMAND_SCHEDULER_HPP_
#define COMMAND_SCHEDULER_HPP_

#include <map>

#include <modm/container/linked_list.hpp>

#include "command.hpp"
#include "mock_macros.hpp"
#include "subsystem.hpp"

namespace aruwlib
{
class Drivers;
namespace control
{
/**
 * Class for handling all the commands you would like to currently run.
 * Interfaces with the Subsystem and Command classes to provide a means
 * of safely scheduling multiple Commands and Subsystems. Checks are
 * provided while scheduling such that multiple commands that require
 * the same subsystem cannot run at the same time. Suppose for example
 * that you have a Command that moves a mechanical arm Subsystem to some
 * position and another Command that moves the same arm to a different
 * position. Obvious issues arise if one attempts to tell the Subsystem
 * to do two things at once. Using this class will disallow these two
 * Commands from being executed at the same time.
 *
 * This class contains a map of Subsystems -> Commands. The Subsystems
 * will be refreshed each time the CommandScheduler is ran. If there
 * are commands associated with the Subsystem, the CommandScheduler will
 * execute these commands as well. Additional less important features
 * are explained in more detail in the function definitions.
 *
 * The goal of this class is for the user to interace directly as
 * little as possible. Aside from calling `run` each time to update the
 * scheduler, the user should be interacting with the Command,
 * ComprisedCommand, Subsystem, and CommandMapper classes to add
 * and remove commands from the scheduler.
 *
 * The main use case will be to be refreshing all the main subsystems running
 * on the robot. To do so, you should call `getMainScheduler()` to access this
 * base scheduler. Here is an example of how to do this:
 *
 * ```
 * // A class that has Command as a base class.
 * CoolSubsystem sub;
 * // A class that has Subsystem as a base class that requires
 * // the subsystem above. In the constructor of the ControlCoolCommand,
 * // you must call `addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem))`;
 * ControlCoolCommand cmd(&sub);
 *
 * CommandScheduler::getMainScheduler().registerSubsystem(&sub);
 * CommandScheduler::getMainScheduler().addCommand(&cmd);
 *
 * while (1) {
 *     // The subsystem will refresh forever and the command until it is not finished.
 *     CommandScheduler::getMainScheduler().run();
 * }
 * ```
 *
 * The second use case for a CommandScheduler is in the ComprisedCommand class.
 * Here, you utilize the CommandScheduler to coordinate multiple commands inside a
 * single command. The usage is exactly the same as using the main CommandScheduler.
 */
class CommandScheduler
{
public:
    CommandScheduler(Drivers* drivers) : drivers(drivers), subsystemToCommandMap() {}
    CommandScheduler(const CommandScheduler&) = delete;
    CommandScheduler& operator=(const CommandScheduler&) = delete;
    mockable ~CommandScheduler() = default;

    /**
     * Calls the `refresh()` function for all Subsystems and the associated
     * `execute()` function for all Commands. A Subsystem is guarenteed to
     * be refreshed no more than one time each time the mainScheduler's run
     * function is called. The same goes for a Command. This includes even if
     * multiple CommandSchedulers are running in ComprisedCommands that have
     * shared Subsystems.
     *
     * If any Subsystem that is in the scheduler does not have a Command
     * controlling it but does have a default command (via the Subsystem's
     * `getDefaultCommand()`), the default command is added to the scheduler.
     *
     * If any Command is finished after execution, the Command is removed from
     * the scheduler. The Command's `end()` function is called, passing in
     * `isInterrupted = false`.
     *
     * @note checks the run time of the scheduler. An error is added to the
     *      error handler if the time is greater than `MAX_ALLOWABLE_SCHEDULER_RUNTIME`
     *      (in microseconds).
     */
    mockable void run();

    /**
     * Removes the given Command completely from the CommandScheduler. This
     * means removing all instances of the command pointer from the Subsystem ->
     * Command map (since a single Subsystem can map to multiple Commands).
     *
     * @param[in] command the Command to remove. Must not be `nullptr`. If the
     *      Command is not in the scheduler, nothing is removed.
     * @param[in] interrupted an argument passed in to the Command's `end()`
     *      function when removing the desired Command.
     */
    mockable void removeCommand(Command* command, bool interrupted);

    /**
     * Adds the given Subsystem to the CommandScheduler.  The subsystem is
     * added with the currently scheduled Command as `nullptr`.
     *
     * @param[in] subsystem the Subsystem to add. Must be not `nullptr` and not
     *      registered already (check via `isSubsystemRegistered()`), otherwise
     *      an error is added to the error handler.
     */
    mockable void registerSubsystem(Subsystem* subsystem);

    /**
     * @param[in] subsystem the subsystem to check
     * @return `true` if the Subsystem is already scheduled, `false` otherwise.
     */
    mockable bool isSubsystemRegistered(Subsystem* subsystem) const;

    /**
     * @return `true` if the CommandScheduler contains the requrested Command.
     *      `false` otherwise.
     */
    mockable bool isCommandScheduled(Command* command) const;

    /**
     * Attempts to add a Command to the scheduler. There are a number of ways this
     * function can fail. If failure does occur, an error will be added to the
     * error handler.
     *
     * These are the following reasons why adding a Command fails:
     * - The commandToAdd is `nullptr`
     * - The commandToAdd has no Subsystem requirements.
     * - The commandToAdd has Subsystems not in the CommandScheduler.
     *
     * If a Command is successfully added to the CommandScheduler, any Subsystems
     * that the commandToAdd requires that have Commands running will be ended
     * (and the interrupted flag for that Command set to `true`).
     *
     * If a Command is successfully added, the Command's `initialize()` function will
     * be called.
     *
     * @param[in] commandToAdd the Command to be added to the scheduler.
     */
    mockable void addCommand(Command* commandToAdd);

private:
    /// Maximum time before we start erroring, in microseconds.
    static constexpr float MAX_ALLOWABLE_SCHEDULER_RUNTIME = 100;

    Drivers* drivers;

    /// a map containing keys of subsystems, pairs of Commands.
    std::map<Subsystem*, Command*> subsystemToCommandMap;

    /**
     * @note this is not a true timestamp. Rather, we use this such that
     *      with multiple CommandSchedulers running with the same Subsystems,
     *      the Subsystems and Commands will be updated only once each time
     *      the `mainScheduler` is ran.
     */
    static uint32_t commandSchedulerTimestamp;
};  // class CommandScheduler

}  // namespace control

}  // namespace aruwlib

#endif  // COMMAND_SCHEDULER_HPP_
