/**
 * This code is part of aruw's repository.
 *
 * A generic extendable class for implementing a command. Each 
 * command is attached to a subsystem. To create a new command,
 * extend the Command class and instantiate the virtual functions
 * in this class. See example_command.hpp for example of this.
 * 
 * Commands can also be comprised of a number of other commands.
 * This is similar to command groups but much less structured.
 * If you are going to do this, please follow the following
 * conventions:
 * - If you are making a comprised command, the comprised command
 *   should operate at a high level. This means a comprised
 *   command should act as a state machine that when it wants
 *   to change the state of the robot, it adds/removes commands
 *   to its command scheduler.
 * - To interface with instances of commands that are a part of 
 *   your comprised command, use the comprisedCommandScheduler
 *   (an instance of a CommandScheduler). You will not need to
 *   register subsystems (this is done when you add a subsystem
 *   requirement to the command), but you can call the add/remove
 *   command.
 */

#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__

#include <set>

namespace aruwlib
{

namespace control
{

class Subsystem;

class Command {
 public:
    Command() :
    prevSchedulerExecuteTimestamp(0)
    {}

    /**
     * Specifies the set of subsystems used by this command.  Two commands cannot
     * use the same subsystem at the same time.  If another command is scheduled
     * that shares a requirement, the command will be interrupted. If no subsystems
     * are required, return an empty set.
     *
     * The generic Command class contains a list of the requrements. The user
     * should add requirements to this list accordingly (typically in the constructor
     * of a class extending the Command class).
     *
     * @return the set of subsystems that are required
     */
    const std::set<Subsystem*>& getRequirements();

    /**
     * Whether the command requires a given subsystem.  Named "hasRequirement"
     * rather than "requires" to avoid confusion with
     *
     * @param requirement the subsystem to inquire about
     * @return whether the subsystem is required
     */
    bool hasRequirement(Subsystem* requirement) const;

    /**
     * Adds the required subsystem to a list of required subsystems
     */
    void addSubsystemRequirement(Subsystem* requirement);

    /**
     * The initial subroutine of a command.  Called once when the command is
     * initially scheduled.
     */
    virtual void initialize(void) = 0;

    /**
     * The main body of a command.  Called repeatedly while the command is
     * scheduled.
     */
    virtual void execute(void) = 0;

    /**
     * The action to take when the command ends.  Called when either the command
     * finishes normally, or when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    virtual void end(bool interrupted) = 0;

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler
     * will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    virtual bool isFinished(void) const = 0;

 private:
    friend class CommandScheduler;

    uint32_t prevSchedulerExecuteTimestamp;

    // contains pointers to const Subsystem pointers that this command requires
    std::set<Subsystem*> commandRequirements;
};

}  // namespace control

}  // namespace aruwlib

#endif
