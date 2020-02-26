#include "turret_manual_command.hpp"
#include "turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
TurretManualCommand::TurretManualCommand(TurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    manualYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    manualPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

bool TurretManualCommand::isFinished() const
{
    return false;
}

void TurretManualCommand::end(bool) {}

void TurretManualCommand::execute()
{
    updateTurretVelocity();
}

void TurretManualCommand::updateTurretVelocity()
{
    pitchVelocityTarget = turretSubsystem->getRemoteYMovement() +
                          turretSubsystem->getMouseXMovement();
    yawVelocityTarget = turretSubsystem->getRemoteXMovement() +
                        turretSubsystem->getMouseYMovement();

    manualPitchPid.update(pitchVelocityTarget - turretSubsystem->getPitchVelocity());
    manualYawPid.update(yawVelocityTarget - turretSubsystem->getYawVelocity());

    turretSubsystem->setPitchMotorOutput(manualPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(manualYawPid.getValue());
}

}  // namespace control

}  // namespace aruwsrc
