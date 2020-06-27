#ifndef __OPEN_HOPPER_COMMAND_HPP__
#define __OPEN_HOPPER_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "hopper_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
class OpenHopperCommand;

class OpenHopperCommand : public aruwlib::control::Command
{
public:
    explicit OpenHopperCommand(HopperSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "open hopper command"; }

private:
    HopperSubsystem* subsystemHopper;
};

}  // namespace control

}  // namespace aruwsrc
#endif
