/**
 * This code is part of aruw's repository.
 * Call this command to extend the y axis on the engineer.
 * This sends a digital out signal to a solenoid, which actuates
 * a piston, used for collecting far bins.
 */

#ifndef __EXTEND_YAXIS_COMMAND__
#define __EXTEND_YAXIS_COMMAND__

#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace engineer
{

class YAxisSubsystem;

class YaxisCommand : public Command
{
 public:
    explicit YaxisCommand(YAxisSubsystem* subsystem);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

    void interrupted();

 private:
    YAxisSubsystem* yAxisSubsystem;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif
