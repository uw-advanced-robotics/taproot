#include <aruwlib/control/command.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include "turret_init_command.hpp"
#include "turret_subsystem.hpp"

namespace aruwsrc
{

namespace turret
{

TurretInitCommand::TurretInitCommand(TurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    initYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    initPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

bool TurretInitCommand::isFinished() const
{
    return fabsf(turretSubsystem->getPitchAngleFromCenter()) < 5.0f
        && fabsf(turretSubsystem->getYawAngleFromCenter()) < 5.0f
        && turretSubsystem->isTurretOnline();
}

void TurretInitCommand::end(bool) {}

void TurretInitCommand::execute()
{
    updateTurretPosition();
}

void TurretInitCommand::updateTurretPosition()
{
    initPitchPid.update(turretSubsystem->getPitchAngle().difference(pitchTargetAngle));
    initYawPid.update(turretSubsystem->getYawAngle().difference(yawTargetAngle));
    turretSubsystem->setPitchMotorOutput(initPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(initYawPid.getValue());
}

}  // namespace turret

}  // namespace aruwsrc
