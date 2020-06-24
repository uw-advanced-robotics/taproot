#include "subsystem.hpp"

#include "command.hpp"

namespace aruwlib
{
namespace control
{
Subsystem::Subsystem() : defaultCommand(nullptr), prevSchedulerExecuteTimestamp(0) {}

void Subsystem::initialize() {}

void Subsystem::setDefaultCommand(Command* command)
{
    if (command != nullptr)
    {
        defaultCommand = command;
    }
}

Command* Subsystem::getDefaultCommand() const { return defaultCommand; }

void Subsystem::refresh() {}

}  // namespace control

}  // namespace aruwlib
