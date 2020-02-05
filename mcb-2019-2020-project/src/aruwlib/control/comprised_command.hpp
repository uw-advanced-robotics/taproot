#ifndef __COMPRISED_COMMAND_HPP__
#define __COMPRISED_COMMAND_HPP__

#include "command.hpp"
#include "command_scheduler.hpp"

namespace aruwlib
{

namespace control
{

class ComprisedCommand : public Command
{
 public:
    ComprisedCommand() : Command(), comprisedCommandScheduler()
    {}

 protected:
    CommandScheduler comprisedCommandScheduler;
};

}  // namespace control

}  // namespace aruwlib

#endif
