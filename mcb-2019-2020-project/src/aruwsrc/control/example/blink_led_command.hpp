#ifndef __BLINK_LED_COMMAND_HPP__
#define __BLINK_LED_COMMAND_HPP__

#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>

#include "example_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
class BlinkLEDCommand : public aruwlib::control::Command
{
public:
    explicit BlinkLEDCommand(aruwsrc::control::ExampleSubsystem* subsystem);

    /**
     * The initial subroutine of a command.  Called once when the command is
     * initially scheduled.
     */
    void initialize() override;

    /**
     * The main body of a command.  Called repeatedly while the command is
     * scheduled.
     */
    void execute() override;

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler
     * will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    bool isFinished() const override;

    const char* getName() const override { return "blink led command"; }

    aruwlib::arch::MilliTimeout completedTimer;

    int refershCounter = 0;
    int startCounter = 0;
};

}  // namespace control

}  // namespace aruwsrc

#endif
