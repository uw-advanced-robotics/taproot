#include "example_comprised_command.hpp"

#include "example_command.hpp"
#include "example_subsystem.hpp"


namespace aruwsrc
{

namespace control
{

ExampleComprisedCommand::ExampleComprisedCommand(ExampleSubsystem* subsystem) :
ComprisedCommand(),
exampleCommand(subsystem, 2000),
otherExampleCommand(subsystem, 500),
switchTimer(2000),
switchCommand(false)
{
    this->addSubsystemRequirement(subsystem);
    this->comprisedCommandScheduler.registerSubsystem(subsystem);
}

void ExampleComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&exampleCommand));
}

void ExampleComprisedCommand::execute() {
    if (switchTimer.execute()) {
        switchTimer.restart(2000);
        if (switchCommand)
        {
            comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&otherExampleCommand));
        }
        else
        {
            comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&exampleCommand));
        }
        switchCommand = !switchCommand;
    }

    this->comprisedCommandScheduler.run();
}

void ExampleComprisedCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(dynamic_cast<Command*>(&exampleCommand), interrupted);
}

}  // namespace control

}  // namespace aruwsrc
