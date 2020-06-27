#ifndef __FRICTION_WHEEL_ROTATE_COMMAND_HPP__
#define __FRICTION_WHEEL_ROTATE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

namespace aruwsrc
{
namespace launcher
{
class FrictionWheelSubsystem;

class FrictionWheelRotateCommand : public aruwlib::control::Command
{
public:
    FrictionWheelRotateCommand(FrictionWheelSubsystem* subsystem, int speed);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "friction wheel rotate command"; }

    static const int16_t DEFAULT_WHEEL_RPM = 6000;

private:
    FrictionWheelSubsystem* frictionWheelSubsystem;

    int speed;
};

}  // namespace launcher

}  // namespace aruwsrc

#endif
