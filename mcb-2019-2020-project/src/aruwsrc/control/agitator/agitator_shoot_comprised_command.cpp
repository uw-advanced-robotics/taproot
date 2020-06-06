#include <aruwlib/control/command_scheduler.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include "agitator_shoot_comprised_command.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace agitator
{

ShootComprisedCommand::ShootComprisedCommand(
    AgitatorSubsystem* agitator,
    float agitatorChangeAngle,
    float maxUnjamAngle,
    uint32_t agitatorDesiredRotateTime,
    uint32_t minAgitatorRotateTime) :
    connectedAgitator(agitator),
    agitatorRotateCommand(
        agitator,
        agitatorChangeAngle,
        agitatorDesiredRotateTime,
        false,
        minAgitatorRotateTime
    ),
    agitatorUnjamCommand(agitator, maxUnjamAngle),
    unjamSequenceCommencing(false)
{
    this->comprisedCommandScheduler.registerSubsystem(agitator);
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(agitator));
}

void ShootComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&agitatorRotateCommand));
    unjamSequenceCommencing = false;
}

void ShootComprisedCommand::execute()
{
    if (connectedAgitator->isAgitatorJammed() && !unjamSequenceCommencing)
    {
        // when the agitator is jammed, add the agitatorUnjamCommand
        // the to scheduler. The rotate forward command will be automatically
        // unscheduled.
        unjamSequenceCommencing = true;
        this->comprisedCommandScheduler.addCommand(
            dynamic_cast<Command*>(&agitatorUnjamCommand));
    }
    this->comprisedCommandScheduler.run();
}

void ShootComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<Command*>(&agitatorUnjamCommand), interrupted);
    this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<Command*>(&agitatorRotateCommand), interrupted);
}

bool ShootComprisedCommand::isFinished() const
{
    return (agitatorRotateCommand.isFinished()
            && !unjamSequenceCommencing)
            || (agitatorUnjamCommand.isFinished()
            && unjamSequenceCommencing);
}

}  // namespace agitator

}  // namespace aruwsrc
