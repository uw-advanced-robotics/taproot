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

#ifndef SUBSYSTEM_HPP_
#define SUBSYSTEM_HPP_

#include <cstdint>

namespace aruwlib
{
class Drivers;
namespace control
{
class Command;

/**
 * A robot subsystem. Subsystems are the basic unit of robot organization in
 * the Command-based framework; they encapsulate low-level hardware objects
 * (motor controllers, sensors, etc) and provide methods through which they can
 * be used by Commands. Subsystems are used by the CommandScheduler's resource
 * management system to ensure multiple robot actions are not "fighting" over
 * the same hardware; Commands that use a subsystem should include that
 * subsystem in their getRequirements() function, and resources used within a
 * subsystem should generally remain encapsulated and not be shared by other
 * parts of the robot.
 *
 * Subsystems must be registered with the scheduler with the
 * CommandScheduler.registerSubsystem() function in order for the
 * refresh() function to be called.
 */
class Subsystem
{
public:
    Subsystem(Drivers* drivers);

    /**
     * Called once when you add the Subsystem to the commandScheduler stored in the
     * Drivers class.
     */
    virtual void initialize();

    /**
     * Sets the default Command of the Subsystem. The default Command will be
     * automatically scheduled when no other Commands are scheduled that require
     * the Subsystem. Default Commands should generally not end on their own, i.e.
     * their `isFinished()` function should always return `false`. Will automatically
     * register this Subsystem with the CommandScheduler if no other Command is
     * scheduled for this Subsystem.
     *
     * @param defaultCommand the default Command to associate with this subsystem
     */
    void setDefaultCommand(Command* defaultCommand);

    /**
     * Gets the default command for this subsystem. Returns `nullptr` if no default
     * command is currently associated with the subsystem.
     *
     * @return the default command associated with this subsystem
     */
    Command* getDefaultCommand() const;

    /**
     * Called in the scheduler's run function assuming this command
     * has been registered with the scheduler. This function should
     * contain code that must be periodically updated and is generic
     * to the subsystem (i.e. updating a control loop generic to this
     * subsystem). This function should not contain command specific
     * control code. When you create a subclass of Subsystem, you
     * should overwrite this virtual function.
     *
     * Must be virtual otherwise scheduler will refer to this function
     * rather than looking in child for this function.
     */
    virtual void refresh();

    virtual bool isHardwareTestComplete() { return hardwareTestsComplete; }

    virtual void setHardwareTestsComplete() { hardwareTestsComplete = true; }

    virtual void runHardwareTests();

    virtual const char* getName();

protected:
    Drivers* drivers;

    bool hardwareTestsComplete = false;

private:
    Command* defaultCommand;

    friend class CommandScheduler;

    uint32_t prevSchedulerExecuteTimestamp;

};  // class Subsystem

}  // namespace control

}  // namespace aruwlib

#endif  // SUBSYSTEM_HPP_
