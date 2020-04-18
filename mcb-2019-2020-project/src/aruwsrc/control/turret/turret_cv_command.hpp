#ifndef __TURRET_CV_COMMAND_H__
#define __TURRET_CV_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include <aruwlib/control/command.hpp>
#include <aruwlib/algorithms/contiguous_float.hpp>

using namespace aruwlib::control;

namespace aruwsrc
{

namespace turret
{

class TurretSubsystem;
class TurretCVCommand : public Command {
 public:
    explicit TurretCVCommand(TurretSubsystem *subsystem);

    void initialize() override;
    bool isFinished() const override;

    void execute() override;

    void end(bool) override;

 private:
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

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAngle;

    modm::Pid<float> CVYawPid;
    modm::Pid<float> CVPitchPid;

    void updateTurretPosition();

    void pitchIncrementAngle(float angle);
    void yawIncrementAngle(float angle);
};

}  // namespace turret

}  // namespace aruwsrc

#endif
