#ifndef EXTEND_XAXIS_COMMAND_HPP_
#define EXTEND_XAXIS_COMMAND_HPP_

#include <aruwlib/control/command.hpp>

namespace aruwsrc
{
namespace engineer
{
class XAxisSubsystem;

/**
 * Call this command to extend the x axis on the engineer.
 * This sends a digital out signal to a solenoid, which actuates
 * a piston, used for collecting far bins.
 */
class ExtendXAxisCommand : public aruwlib::control::Command
{
public:
    explicit ExtendXAxisCommand(XAxisSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    XAxisSubsystem* xAxisSubsystem;
};  // class ExtendXAxisCommand

}  // namespace engineer

}  // namespace aruwsrc

#endif  // EXTEND_XAXIS_COMMAND_HPP_
