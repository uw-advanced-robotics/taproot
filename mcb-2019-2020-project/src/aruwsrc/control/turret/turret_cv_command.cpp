#include "turret_cv_command.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/communication/remote.hpp>
#include <aruwlib/communication/serial/xavier_serial.hpp>

using namespace aruwlib;
using namespace aruwlib::serial;
namespace aruwsrc
{
namespace turret
{
TurretCVCommand::TurretCVCommand(TurretSubsystem *subsystem)
    : turretSubsystem(subsystem),
      yawTargetAngle(TurretSubsystem::TURRET_START_ANGLE, 0.0f, 360.0f),
      pitchTargetAngle(TurretSubsystem::TURRET_START_ANGLE, 0.0f, 360.0f),
      yawPid(
          YAW_P,
          YAW_I,
          YAW_D,
          YAW_MAX_ERROR_SUM,
          YAW_MAX_OUTPUT,
          YAW_Q_DERIVATIVE_KALMAN,
          YAW_R_DERIVATIVE_KALMAN,
          YAW_Q_PROPORTIONAL_KALMAN,
          YAW_R_PROPORTIONAL_KALMAN),
      pitchPid(
          PITCH_P,
          PITCH_I,
          PITCH_D,
          PITCH_MAX_ERROR_SUM,
          PITCH_MAX_OUTPUT,
          PITCH_Q_DERIVATIVE_KALMAN,
          PITCH_R_DERIVATIVE_KALMAN,
          PITCH_Q_PROPORTIONAL_KALMAN,
          PITCH_R_PROPORTIONAL_KALMAN),
      sendRequestTimer(TIME_BETWEEN_CV_REQUESTS)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem *>(subsystem));
}

void TurretCVCommand::initialize()
{
    sendRequestTimer.restart(TIME_BETWEEN_CV_REQUESTS);
    Drivers::xavierSerial.beginTargetTracking();
    yawPid.reset();
    pitchPid.reset();
}

void TurretCVCommand::execute()
{
    XavierSerial::TurretAimData cvData;
    if (Drivers::xavierSerial.getLastAimData(&cvData))
    {
        if (cvData.hasTarget)
        {
            turretSubsystem->setYawTarget(cvData.yaw);
            turretSubsystem->setPitchTarget(cvData.pitch);
        }
    }
    else if (sendRequestTimer.isExpired())
    {
        Drivers::xavierSerial.beginTargetTracking();
    }
    runYawPositionController();
    runPitchPositionController();
}

// NOLINTNEXTLINE
void TurretCVCommand::end(bool) { Drivers::xavierSerial.stopTargetTracking(); }

void TurretCVCommand::runYawPositionController()
{
    // position controller based on gimbal angle
    float positionControllerError =
        turretSubsystem->getYawAngle().difference(turretSubsystem->getYawTarget());
    float pidOutput =
        yawPid.runController(positionControllerError, turretSubsystem->getYawVelocity());

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretCVCommand::runPitchPositionController()
{
    // position controller based on turret pitch gimbal
    float positionControllerError =
        turretSubsystem->getPitchAngle().difference(turretSubsystem->getPitchTarget());
    float pidOutput =
        pitchPid.runController(positionControllerError, turretSubsystem->getPitchVelocity());

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

}  // namespace turret

}  // namespace aruwsrc
