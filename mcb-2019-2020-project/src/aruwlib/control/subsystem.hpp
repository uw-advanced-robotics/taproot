/**
 * This code is part of aruw's repository.
 *
 * A robot subsystem. Subsystems are the basic unit of robot organization in
 * the Command-based framework; they encapsulate low-level hardware objects
 * (motor controllers, sensors, etc) and provide methods through which they can
 * be used by Commands. Subsystems are used by the CommandScheduler's resource
 * management system to ensure multiple robot actions are not "fighting" over
 * the same hardware; Commands that use a subsystem should include that
 * subsystem in their getRequirements() method, and resources used within a
 * subsystem should generally remain encapsulated and not be shared by other
 * parts of the robot.
 *
 * <p>Subsystems must be registered with the scheduler with the
 * CommandScheduler.registerSubsystem() method in order for the
 * refresh() method to be called. It is recommended that this method be called
 * from the constructor of users' Subsystem implementations. The
 * SubsystemBase class offers a simple base for user implementations
 * that handles this.
 */

#ifndef __SUBSYSTEM_HPP__
#define __SUBSYSTEM_HPP__

#include <cstdint>

namespace aruwlib
{

namespace control
{

class Command;

class Subsystem {
 public:
    Subsystem();

    /**
     * Sets the default Command of the subsystem. The default command will be
     * automatically scheduled when no other commands are scheduled that require
     * the subsystem. Default commands should generally not end on their own, i.e.
     * their isFinished() method should always return false. Will automatically
     * register this subsystem with the CommandScheduler if no other command is
     * scheduled for this subsystem.
     *
     * @param defaultCommand the default command to associate with this subsystem
     */
    void setDefaultCommand(Command* defaultCommand);

    /**
     * Gets the default command for this subsystem. Returns null if no default
     * command is currently associated with the subsystem.
     *
     * @return the default command associated with this subsystem
     */
    Command* getDefaultCommand(void) const;

    /**
     * Called in the scheduler's run method assuming this command
     * has been registered with the scheduler. This method should
     * contain code that must be periodically updated and is generic
     * to the subsystem (i.e. updating a control loop generic to this
     * subsystem). This method should not contain command specific
     * control code. When you create a subclass of Subsystem, you
     * should overwrite this virtual method.
     * 
     * Must be virtual otherwise scheduler will refer to this method
     * rather than looking in child for this method.
     */
    virtual void refresh(void);

    uint32_t prevSchedulerExecuteTimestamp;

 private:
    Command* defaultCommand;
};

}  // namespace control

}  // namespace aruwlib

#endif
