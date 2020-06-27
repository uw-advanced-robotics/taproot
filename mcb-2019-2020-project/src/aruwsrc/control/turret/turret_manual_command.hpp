#ifndef __TURRET_MANUAL_COMMAND_H__
#define __TURRET_MANUAL_COMMAND_H__

#include <aruwlib/control/command.hpp>
#include <modm/math/filter/pid.hpp>

namespace aruwsrc
{
namespace turret
{
class TurretSubsystem;
class TurretManualCommand : public aruwlib::control::Command
{
public:
    explicit TurretManualCommand(TurretSubsystem *subsystem);

    void initialize() override {}
    bool isFinished() const override;

    void execute() override;
    void end(bool) override;

    const char *getName() const override { return "turret manual command"; }

private:
    const float USER_INPUT_SCALAR = 50.0f;

    const float YAW_P = 1.0f;
    const float YAW_I = 0.0f;
    const float YAW_D = 0.0f;
    const float YAW_MAX_ERROR_SUM = 0.0f;
    const float YAW_MAX_OUTPUT = 16000;

    const float PITCH_P = 1.0f;
    const float PITCH_I = 0.0f;
    const float PITCH_D = 0.0f;
    const float PITCH_MAX_ERROR_SUM = 0.0f;
    const float PITCH_MAX_OUTPUT = 16000;

    TurretSubsystem *turretSubsystem;
    modm::Pid<float> manualYawPid;
    modm::Pid<float> manualPitchPid;

    float yawVelocityTarget = 0;
    float pitchVelocityTarget = 0;

    void updateTurretVelocity();
};

}  // namespace turret

}  // namespace aruwsrc

#endif
