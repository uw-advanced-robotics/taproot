#ifndef SUBSYSTEM_HPP_
#define SUBSYSTEM_HPP_

#include <cstdint>

namespace aruwlib
{
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
    Subsystem();

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

private:
    Command* defaultCommand;

    friend class CommandScheduler;

    uint32_t prevSchedulerExecuteTimestamp;
};  // class Subsystem

}  // namespace control

}  // namespace aruwlib

#endif  // SUBSYSTEM_HPP_
