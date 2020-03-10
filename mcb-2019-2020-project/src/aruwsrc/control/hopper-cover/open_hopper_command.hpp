#ifndef __OPEN_HOPPER_COMMAND_HPP__
#define __OPEN_HOPPER_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
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

    void initialize();

    void execute();

    void end(bool);

    bool isFinished() const;

 private:
    HopperSubsystem* subsystemHopper;
};

}  // namespace control

}  // namespace aruwsrc
#endif
