#ifndef __EXAMPLE_COMPRISED_COMAND_HPP__
#define __EXAMPLE_COMPRISED_COMAND_HPP__

#include <aruwlib/control/comprised_command.hpp>
#include "example_command.hpp"
#include "example_subsystem.hpp"

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

    aruwlib::arch::MilliTimeout switchTimer;

    bool switchCommand;
};

}  // namespace control

}  // namespace aruwsrc

#endif
