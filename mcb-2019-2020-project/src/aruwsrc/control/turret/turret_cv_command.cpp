#include <aruwlib/communication/remote.hpp>
#include "turret_cv_command.hpp"
#include "turret_subsystem.hpp"

namespace aruwsrc
{

namespace turret
{

TurretCVCommand::TurretCVCommand(TurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    yawTargetAngle(0, 0, 360),
    pitchTargetAngle(0, 0, 360),
    CVYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    CVPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

void TurretCVCommand::initialize()
{
    // add xavier stuff here
}

bool TurretCVCommand::isFinished() const
{
    return false;
}

void TurretCVCommand::end(bool) {}

void TurretCVCommand::execute()
{
    updateTurretPosition();
}

void TurretCVCommand::pitchIncrementAngle(float angle)
{
    pitchTargetAngle.shiftValue(angle);
}

void TurretCVCommand::yawIncrementAngle(float angle)
{
    yawTargetAngle.shiftValue(angle);
}

void TurretCVCommand::updateTurretPosition()
{
    CVPitchPid.update(pitchTargetAngle.difference(turretSubsystem->getPitchAngle()));
    CVYawPid.update(yawTargetAngle.difference(turretSubsystem->getYawAngle()));

    turretSubsystem->setPitchMotorOutput(CVPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(CVYawPid.getValue());
}

}  // namespace turret

}  // namespace aruwsrc
