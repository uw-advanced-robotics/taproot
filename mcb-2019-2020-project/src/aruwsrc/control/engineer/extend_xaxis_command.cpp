#include "extend_xaxis_command.hpp"

#include <aruwlib/control/subsystem.hpp>

#include "xaxis_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
ExtendXAxisCommand::ExtendXAxisCommand(XAxisSubsystem* subsystem)
    : Command(),
      xAxisSubsystem(subsystem)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
}

void ExtendXAxisCommand::initialize()
{
    xAxisSubsystem->setExtended(true);  // default movement is "not extended"
}

void ExtendXAxisCommand::execute() {}

void ExtendXAxisCommand::end(bool) { xAxisSubsystem->setExtended(false); }

bool ExtendXAxisCommand::isFinished() const { return false; }
}  // namespace engineer

}  // namespace aruwsrc
