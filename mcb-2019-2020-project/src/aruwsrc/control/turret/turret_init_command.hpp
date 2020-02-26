#ifndef __TURRET_INIT_COMMAND_H__
#define __TURRET_INIT_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/contiguous_float.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem;
class TurretInitCommand : public Command {
 public:
    explicit TurretInitCommand(TurretSubsystem *subsystem);

    void initialize() {}
    bool isFinished() const;

    void execute();
    void end(bool);

 private:
    const float YAW_P = 300.0f;
    const float YAW_I = 0.0f;
    const float YAW_D = 100.0f;
    const float YAW_MAX_ERROR_SUM = 0.0f;
    const float YAW_MAX_OUTPUT = 16000;

    const float PITCH_P = 300.0f;
    const float PITCH_I = 0.0f;
    const float PITCH_D = 100.0f;
    const float PITCH_MAX_ERROR_SUM = 0.0f;
    const float PITCH_MAX_OUTPUT = 16000;

    const float pitchTargetAngle = 90.0f;
    const float yawTargetAngle = 90.0f;

    TurretSubsystem *turretSubsystem;

    modm::Pid<float> initYawPid;
    modm::Pid<float> initPitchPid;

    void updateTurretPosition(void);
};

}  // namespace control

}  // namespace aruwsrc

#endif
