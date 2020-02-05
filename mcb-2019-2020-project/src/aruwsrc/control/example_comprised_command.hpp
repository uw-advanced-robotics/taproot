#ifndef __HPP__
#define __HPP__

#include "example_command.hpp"
#include "example_subsystem.hpp"
#include "src/aruwlib/control/comprised_command.hpp"

namespace aruwsrc
{

namespace control
{

class ExampleComprisedCommand : public ComprisedCommand
{
 public:
    explicit ExampleComprisedCommand(ExampleSubsystem* subsystem);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const override
    {
       return false;
    }

 private:
    ExampleCommand exampleCommand;

    ExampleCommand otherExampleCommand;

    modm::ShortTimeout switchTimer;

    bool switchCommand;
};

}  // namespace control

}  // namespace aruwsrc

#endif
