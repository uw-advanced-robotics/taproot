#include "extend_xaxis_command.hpp"
#include "xaxis_subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{
    XaxisCommand::XaxisCommand(XAxisSubsystem* subsystem)
        : Command(), xAxisSubsystem(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void XaxisCommand::initialize(void)
    {
        xAxisSubsystem->setXAxisExtended(true);  // default movement is "not extended"
    }

    void XaxisCommand::execute(void)
    {
        xAxisSubsystem->setXAxisExtended(true);
    }

    void XaxisCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            xAxisSubsystem->setXAxisExtended(false);
        }
        xAxisSubsystem->setXAxisExtended(false);
    }

    bool XaxisCommand::isFinished(void) const
    {
        return false;
    }

    void XaxisCommand::interrupted(void)
    {}
}  // namespace engineer

}  // namespace aruwsrc
