#include "extend_yaxis_command.hpp"
#include "yaxis_subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{
    YaxisCommand::YaxisCommand(YAxisSubsystem* subsystem)
        : Command(), yAxisSubsystem(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void YaxisCommand::initialize(void)
    {
        yAxisSubsystem->setYAxisExtended(true);  // default movement is "not extended"
    }

    void YaxisCommand::execute(void)
    {
        yAxisSubsystem->setYAxisExtended(true);
    }

    void YaxisCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            yAxisSubsystem->setYAxisExtended(false);
        }
        yAxisSubsystem->setYAxisExtended(false);
    }

    bool YaxisCommand::isFinished(void) const
    {
        return false;
    }

    void YaxisCommand::interrupted(void)
    {}
}  // namespace engineer

}  // namespace aruwsrc
