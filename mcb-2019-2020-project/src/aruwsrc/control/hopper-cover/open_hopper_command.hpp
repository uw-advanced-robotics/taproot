#ifndef __OPEN_HOPPER_COMMAND_HPP__
#define __OPEN_HOPPER_COMMAND_HPP__

#include <aruwlib/control/command.hpp>
#include "hopper_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class OpenHopperCommand;

class OpenHopperCommand : public Command
{
 public:
    explicit OpenHopperCommand(HopperSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

 private:
    HopperSubsystem* subsystemHopper;
};

}  // namespace control

}  // namespace aruwsrc
#endif
