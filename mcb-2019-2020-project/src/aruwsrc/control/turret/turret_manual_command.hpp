#ifndef __TURRET_MANUAL_COMMAND_H__
#define __TURRET_MANUAL_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem;
class TurretManualCommand : public Command {
 public:
    explicit TurretManualCommand(TurretSubsystem *subsystem);

    void initialize() {}
    bool isFinished() const;

    void execute();
    void end(bool);

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

}  // namespace control

}  // namespace aruwsrc

#endif
