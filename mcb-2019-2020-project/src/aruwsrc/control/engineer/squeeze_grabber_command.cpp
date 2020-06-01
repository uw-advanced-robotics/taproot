#include "squeeze_grabber_command.hpp"

#include <aruwlib/control/subsystem.hpp>

#include "grabber_subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{
SqueezeGrabberCommand::SqueezeGrabberCommand(GrabberSubsystem* subsystem)
    : Command(), grabber(subsystem)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(grabber));
}

void SqueezeGrabberCommand::initialize()
{
    grabber->setSqueezed(true);
}

void SqueezeGrabberCommand::execute()
{}

// NOLINTNEXTLINE
void SqueezeGrabberCommand::end(bool)
{
    grabber->setSqueezed(false);
}

bool SqueezeGrabberCommand::isFinished() const
{
    return false;
}
}  // namespace engineer

}  // namespace aruwsrc
