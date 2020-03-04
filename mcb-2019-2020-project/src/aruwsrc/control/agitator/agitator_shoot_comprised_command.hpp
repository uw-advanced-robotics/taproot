#ifndef __SHOOT_COMPRISED_COMMAND_HPP__
#define __SHOOT_COMPRISED_COMMAND_HPP__

#include "src/aruwlib/control/comprised_command.hpp"
#include "agitator_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"

namespace aruwsrc
{

namespace agitator
{

class ShootComprisedCommand : public aruwlib::control::ComprisedCommand
{
 public:
    ShootComprisedCommand(
        AgitatorSubsystem* agitator,
        float agitatorChangeAngle,
        float maxUnjamAngle,
        uint32_t agitatorDesiredRotateTime,
        uint32_t minAgitatorRotateTime
    );

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

 private:
    AgitatorSubsystem* connectedAgitator;

    AgitatorRotateCommand agitatorRotateCommand;

    AgitatorUnjamCommand agitatorUnjamCommand;

    bool unjamSequenceCommencing;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
