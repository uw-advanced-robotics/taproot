#include "turret_world_relative_position_command.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/control/control_operator_interface.hpp"

using namespace aruwlib::sensors;
using namespace aruwlib;

namespace aruwsrc
{

namespace control
{

TurretWorldRelativePositionCommand::TurretWorldRelativePositionCommand(
    TurretSubsystem *subsystem, chassis::ChassisSubsystem *chassis
) :
    turretSubsystem(subsystem),
    chassisSubsystem(chassis),
    yawTargetAngle(TurretSubsystem::TURRET_START_ANGLE, 0.0f, 360.0f),
    currValueImuYawGimbal(0.0f, 0.0f, 360.0f),
    imuInitialYaw(0.0f),
    yawPid(
        YAW_P,
        YAW_I,
        YAW_D,
        YAW_MAX_ERROR_SUM,
        YAW_MAX_OUTPUT,
        YAW_Q_DERIVATIVE_KALMAN,
        YAW_R_DERIVATIVE_KALMAN,
        YAW_Q_PROPORTIONAL_KALMAN,
        YAW_R_PROPORTIONAL_KALMAN
    ),
    pitchPid(
        PITCH_P,
        PITCH_I,
        PITCH_D,
        PITCH_MAX_ERROR_SUM,
        PITCH_MAX_OUTPUT,
        PITCH_Q_DERIVATIVE_KALMAN,
        PITCH_R_DERIVATIVE_KALMAN,
        PITCH_Q_PROPORTIONAL_KALMAN,
        PITCH_R_PROPORTIONAL_KALMAN
    )
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
}

void TurretWorldRelativePositionCommand::initialize()
{
    imuInitialYaw = Mpu6500::getImuAttitude().yaw;
    yawPid.reset();
    pitchPid.reset();
    yawTargetAngle.setValue(turretSubsystem->getYawTarget());
}

void TurretWorldRelativePositionCommand::execute()
{
    runYawPositionController();
    runPitchPositionController();
}

void TurretWorldRelativePositionCommand::runYawPositionController()
{
    turretSubsystem->updateCurrentTurretAngles();

    yawTargetAngle.shiftValue(USER_YAW_INPUT_SCALAR
            * aruwlib::control::ControlOperatorInterface::getTurretYawInput());

    // project target angle in world relative to chassis relative to limit the value
    turretSubsystem->setYawTarget(projectWorldRelativeYawToChassisFrame(
            yawTargetAngle.getValue(), imuInitialYaw));

    // project angle that is limited by the subsystem to world relative again to run the controller
    yawTargetAngle.setValue(projectChassisRelativeYawToWorldRelative(
            turretSubsystem->getYawTarget(), imuInitialYaw));

    currValueImuYawGimbal.setValue(
            projectChassisRelativeYawToWorldRelative(
                    turretSubsystem->getYawAngle().getValue(), imuInitialYaw));

    // position controller based on imu and yaw gimbal angle
    float positionControllerError = currValueImuYawGimbal.difference(yawTargetAngle);
    float pidOutput = yawPid.runController(positionControllerError,
            turretSubsystem->getYawVelocity() + Mpu6500::getGz());

    pidOutput += turretSubsystem->yawFeedForwardCalculation(
            chassisSubsystem->getChassisDesiredRotation());

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretWorldRelativePositionCommand::runPitchPositionController()
{
    // limit the yaw min and max angles
    turretSubsystem->setPitchTarget(turretSubsystem->getPitchTarget()
            + USER_PITCH_INPUT_SCALAR
            * aruwlib::control::ControlOperatorInterface::getTurretPitchInput());

    // position controller based on turret pitch gimbal and imu data
    float positionControllerError = turretSubsystem->getPitchAngle().difference(
            turretSubsystem->getPitchTarget());

    float pidOutput = pitchPid.runController(positionControllerError,
        turretSubsystem->getPitchVelocity());

    // gravity compensation
    pidOutput += PITCH_GRAVITY_COMPENSATION_KP * cosf(aruwlib::algorithms::degreesToRadians(
            turretSubsystem->getPitchAngleFromCenter()));

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

// NOLINTNEXTLINE
void TurretWorldRelativePositionCommand::end(bool)
{
    turretSubsystem->setYawTarget(projectWorldRelativeYawToChassisFrame(yawTargetAngle.getValue(),
            imuInitialYaw));
}

float TurretWorldRelativePositionCommand::projectChassisRelativeYawToWorldRelative(
    float yawAngle,
    float imuInitialAngle
) {
    return yawAngle + aruwlib::sensors::Mpu6500::getImuAttitude().yaw - imuInitialAngle;
}

float TurretWorldRelativePositionCommand::projectWorldRelativeYawToChassisFrame(
    float yawAngle,
    float imuInitialAngle
) {
    return yawAngle - aruwlib::sensors::Mpu6500::getImuAttitude().yaw + imuInitialAngle;
}

}  // namespace control

}  // namespace aruwsrc
