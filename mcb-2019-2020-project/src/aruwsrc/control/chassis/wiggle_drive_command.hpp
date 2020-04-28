#ifndef __WIGGLE_DRIVE_COMMAND_HPP__
#define __WIGGLE_DRIVE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "chassis_subsystem.hpp"

using namespace aruwlib::control;
using namespace aruwsrc::turret;

namespace aruwsrc
{

namespace chassis
{

class WiggleDriveCommand : public Command {
 public:
    explicit WiggleDriveCommand(ChassisSubsystem* chassis, TurretSubsystem* turret)
    : chassis(chassis), turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(chassis));
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

 private:
    static constexpr float WIGGLE_PERIOD = 1600.0f;
    static constexpr float WIGGLE_MAX_ROTATE_ANGLE = 60.0f;
    static constexpr float WIGGLE_ROTATE_KP = -250.0f;
    static constexpr float TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING = 0.5f;
    static constexpr float WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR = 10.0f;

    ChassisSubsystem* chassis;
    TurretSubsystem* turret;

    uint32_t timeOffset = 0;
    float startTimeForAngleOffset = 0.0f;
    bool outOfCenter = false;

    // sin curve to determine angle to rotate to based on current "time"
    float wiggleSin(float time);
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
