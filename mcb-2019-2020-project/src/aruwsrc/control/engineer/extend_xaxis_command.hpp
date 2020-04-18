/**
 * This code is part of aruw's repository.
 * Call this command to extend the x axis on the engineer.
 * This sends a digital out signal to a solenoid, which actuates
 * a piston, used for collecting far bins.
 */

#ifndef __EXTEND_XAXIS_COMMAND__
#define __EXTEND_XAXIS_COMMAND__

#include <aruwlib/control/command.hpp>

using namespace aruwlib::control;

namespace aruwsrc
{

namespace engineer
{

class XAxisSubsystem;

class XaxisCommand : public Command
{
 public:
    explicit XaxisCommand(XAxisSubsystem* subsystem);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

 private:
    XAxisSubsystem* xAxisSubsystem;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif
