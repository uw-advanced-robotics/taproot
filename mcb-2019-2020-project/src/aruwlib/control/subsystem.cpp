#include "subsystem.hpp"
#include "command.hpp"
#include "command_scheduler.hpp"

namespace aruwlib
{

namespace control
{
    Subsystem::Subsystem() : prevSchedulerExecuteTimestamp(0), defaultCommand(nullptr)
    {}

    void Subsystem::setDefaultCommand(Command* command)
    {
        if (command != nullptr)
        {
            defaultCommand = command;
        }
    }

    Command* Subsystem::getDefaultCommand() const
    {
        return defaultCommand;
    }

    void Subsystem::refresh(void) {}

}  // namespace control

}  // namespace aruwlib
