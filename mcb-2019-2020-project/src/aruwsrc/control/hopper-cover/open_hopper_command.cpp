#include "open_hopper_command.hpp"


namespace aruwsrc
{

namespace control
{
    OpenHopperCommand::OpenHopperCommand(HopperSubsystem* subsystem)
        : Command(), subsystemHopper(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
    }

    void OpenHopperCommand::initialize()
    {
        subsystemHopper->setOpen();
    }

    // set the hopper servo to the open position
    void OpenHopperCommand::execute()
    {}

    // set the hopper servo to the close position
    // NOLINTNEXTLINE
    void OpenHopperCommand::end(bool)
    {
        subsystemHopper->setClose();
    }

    bool OpenHopperCommand::isFinished() const
    {
        return false;
    }

}  // namespace control

}  // namespace aruwsrc
