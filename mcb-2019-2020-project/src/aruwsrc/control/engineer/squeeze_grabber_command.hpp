#ifndef SQUEEZE_GRABBER_COMMAND_HPP_
#define SQUEEZE_GRABBER_COMMAND_HPP_

#include <aruwlib/control/command.hpp>

namespace aruwsrc
{
namespace engineer
{
class GrabberSubsystem;

class SqueezeGrabberCommand : public aruwlib::control::Command
{
public:
    explicit SqueezeGrabberCommand(GrabberSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    GrabberSubsystem* grabber;
};  // class SqueezeGrabberCommand

}  // namespace engineer

}  // namespace aruwsrc
#endif  // SQUEEZE_GRABBER_COMMAND_HPP_
