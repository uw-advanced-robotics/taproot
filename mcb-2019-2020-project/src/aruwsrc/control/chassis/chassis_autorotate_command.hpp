#ifndef __CHASSIS_AUTOROTATE_COMMAND_HPP__
#define __CHASSIS_AUTOROTATE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "chassis_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"

#include "src/aruwlib/motor/dji_motor.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace chassis
{

class ChassisAutorotateCommand : public Command
{
 public:
    explicit ChassisAutorotateCommand(ChassisSubsystem* chassis,
                                     aruwsrc::control::TurretSubsystem const* turret) :
                                     chassis(chassis),
                                     turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(chassis));
    }

    void initialize();

    void execute();

    void end(bool);

    bool isFinished() const;

 private:
    static constexpr float CHASSIS_AUTOROTATE_PID_KP = -85.0f;

    ChassisSubsystem* chassis;
    aruwsrc::control::TurretSubsystem const* turret;
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
