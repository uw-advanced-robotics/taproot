#ifndef __COMPRISED_COMMAND_HPP__
#define __COMPRISED_COMMAND_HPP__

#include "command.hpp"
#include "command_scheduler.hpp"

namespace aruwlib
{
namespace control
{
/**
 * A class with all the features of a Command but with the addition of
 * a CommandScheduler that can be used to schedule multiple
 * Commands inside a single Command. If you are making a comprised command,
 * operations in this Command should operate at a high level. In essence,
 * a comprised acts as a vessel for a state machine that when it wants
 * to change the state of the robot, it adds/removes commands to its
 * command scheduler instead of directly interacting with a subsystem.
 *
 * For example, consider this use case: You have a Command that actuates
 * a piston to grab something and another Command that flips a wrist that
 * has the piston out. It would be nice to reuse these Commands and make
 * a Command that flips the wrist out and then actuates the piston in quick
 * succession. To do so, you can create a ComprisedCommand that consists of
 * the two Commands described above. In this ComprisedCommand, first
 * schedule the Command that flips the wrist out, then when that Command
 * is done, schedule the Command that actuates the piston.
 *
 * When you are using this the `comprisedCommandScheduler`, be sure to
 * register Subsystems and add Subsystem dependencies for the Commands that
 * will be added to the scheduler.
 */
class ComprisedCommand : public Command
{
public:
    ComprisedCommand() : Command(), comprisedCommandScheduler() {}

protected:
    CommandScheduler comprisedCommandScheduler;
};

}  // namespace control

}  // namespace aruwlib

#endif
