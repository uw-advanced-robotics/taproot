#ifndef __CHASSIS_AUTOROTATE_COMMAND_HPP__
#define __CHASSIS_AUTOROTATE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
class ChassisAutorotateCommand : public aruwlib::control::Command
{
public:
    explicit ChassisAutorotateCommand(
        ChassisSubsystem* chassis,
        aruwsrc::turret::TurretSubsystem const* turret)
        : chassis(chassis),
          turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
    }

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis autorotate command"; }

private:
    static constexpr float CHASSIS_AUTOROTATE_PID_KP = -85.0f;

    ChassisSubsystem* chassis;
    aruwsrc::turret::TurretSubsystem const* turret;
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
