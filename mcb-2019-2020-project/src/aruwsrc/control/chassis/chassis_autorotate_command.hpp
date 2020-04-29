#ifndef __CHASSIS_AUTOROTATE_COMMAND_HPP__
#define __CHASSIS_AUTOROTATE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "chassis_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace chassis
{

class ChassisAutorotateCommand : public Command
{
 public:
    explicit ChassisAutorotateCommand(ChassisSubsystem* chassis,
                                     aruwsrc::turret::TurretSubsystem const* turret) :
                                     chassis(chassis),
                                     turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(chassis));
    }

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

 private:
    static constexpr float CHASSIS_AUTOROTATE_PID_KP = -85.0f;

    ChassisSubsystem* chassis;
    aruwsrc::turret::TurretSubsystem const* turret;
};

}  // namespace chassis

}  // namespace aruwsrc

#endif