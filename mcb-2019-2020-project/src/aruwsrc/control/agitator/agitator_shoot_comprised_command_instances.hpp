#ifndef __AGITATOR_SHOOT_COMPRISED_COMMANDS_HPP__
#define __AGITATOR_SHOOT_COMPRISED_COMMANDS_HPP__

#include "agitator_shoot_comprised_command.hpp"

namespace aruwsrc
{
namespace agitator
{
class ShootFastComprisedCommand : public ShootComprisedCommand
{
public:
    explicit ShootFastComprisedCommand(AgitatorSubsystem* agitator17mm)
        : ShootComprisedCommand(
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              50,
              20)
    {
    }
};

class ShootSlowComprisedCommand : public ShootComprisedCommand
{
public:
    explicit ShootSlowComprisedCommand(AgitatorSubsystem* agitator17mm)
        : ShootComprisedCommand(
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              300,
              100)
    {
    }
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
