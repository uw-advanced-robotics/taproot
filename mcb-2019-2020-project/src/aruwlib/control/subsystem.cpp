#include "subsystem.hpp"
#include "command.hpp"
#include "command_scheduler.hpp"

namespace aruwlib
{

namespace control
{
    Subsystem::Subsystem() : defaultCommand(CommandScheduler::defaultNullCommand)
    {}

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
