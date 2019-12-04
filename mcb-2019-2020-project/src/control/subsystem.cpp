#include "src/control/subsystem.hpp"
#include "src/control/command.hpp"
#include "command_scheduler.hpp"

namespace aruwlib
{

namespace control
{
    Subsystem::Subsystem() : defaultCommand()
    {
        defaultCommand = CommandScheduler::defaultNullCommand;
    }

    void Subsystem::setDefaultCommand(modm::SmartPointer command)
    {
        defaultCommand = command;
    }

    modm::SmartPointer Subsystem::getDefaultCommand() const
    {
        return defaultCommand;
    }

    void Subsystem::refresh(void) {}

}  // namespace control

}  // namespace aruwlib
