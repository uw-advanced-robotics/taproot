#ifndef __BLINK_LED_COMMAND_HPP__
#define __BLINK_LED_COMMAND_HPP__

#include <modm/processing/timer/timeout.hpp>
#include "src/aruwlib/control/command.hpp"
#include "example_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class BlinkLEDCommand : public Command
{
 public:
    explicit BlinkLEDCommand(aruwsrc::control::ExampleSubsystem* subsystem);

    /**
      * The initial subroutine of a command.  Called once when the command is
      * initially scheduled.
      */
    void initialize(void);

    /**
      * The main body of a command.  Called repeatedly while the command is
      * scheduled.
      */
    void execute(void);

    /**
      * The action to take when the command ends.  Called when either the command
      * finishes normally, or when it interrupted/canceled.
      *
      * @param interrupted whether the command was interrupted/canceled
      */
    void end(bool interrupted);

    /**
      * Whether the command has finished.  Once a command finishes, the scheduler
      * will call its end() method and un-schedule it.
      *
      * @return whether the command has finished.
      */
    bool isFinished(void) const;

    void interrupted(void);

    /**
      * Whether the given command should run when the robot is disabled.  Override
      * to return true if the command should run when disabled.
      *
      * @return whether the command should run when the robot is disabled
      */
    bool runsWhenDisabled(void);

    modm::ShortTimeout completedTimer;

    int refershCounter = 0;
    int endCounter = 0;
    int startCounter = 0;
};

}  // namespace control

}  // namespace aruwsrc

#endif
