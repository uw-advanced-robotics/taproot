/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "turret_world_relative_position_command.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>

using namespace aruwlib::sensors;

namespace aruwsrc
{
namespace turret
{
TurretWorldRelativePositionCommand::TurretWorldRelativePositionCommand(
    aruwlib::Drivers *drivers,
    TurretSubsystem *subsystem,
    chassis::ChassisSubsystem *chassis)
    : drivers(drivers),
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
          PITCH_R_PROPORTIONAL_KALMAN)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem *>(subsystem));
}

void TurretWorldRelativePositionCommand::initialize()
{
    imuInitialYaw = drivers->mpu6500.getYaw();
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

    yawTargetAngle.shiftValue(
        USER_YAW_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretYawInput());

    // project target angle in world relative to chassis relative to limit the value
    turretSubsystem->setYawTarget(
        projectWorldRelativeYawToChassisFrame(yawTargetAngle.getValue(), imuInitialYaw));

    // project angle that is limited by the subsystem to world relative again to run the controller
    yawTargetAngle.setValue(
        projectChassisRelativeYawToWorldRelative(turretSubsystem->getYawTarget(), imuInitialYaw));

    currValueImuYawGimbal.setValue(projectChassisRelativeYawToWorldRelative(
        turretSubsystem->getYawAngle().getValue(),
        imuInitialYaw));

    // position controller based on imu and yaw gimbal angle
    float positionControllerError = currValueImuYawGimbal.difference(yawTargetAngle);
    float pidOutput = yawPid.runController(
        positionControllerError,
        turretSubsystem->getYawVelocity() + drivers->mpu6500.getGz());

    pidOutput +=
        turretSubsystem->yawFeedForwardCalculation(chassisSubsystem->getChassisDesiredRotation());

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretWorldRelativePositionCommand::runPitchPositionController()
{
    // limit the yaw min and max angles
    turretSubsystem->setPitchTarget(
        turretSubsystem->getPitchTarget() +
        USER_PITCH_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput());

    // position controller based on turret pitch gimbal and imu data
    float positionControllerError =
        turretSubsystem->getPitchAngle().difference(turretSubsystem->getPitchTarget());

    float pidOutput =
        pitchPid.runController(positionControllerError, turretSubsystem->getPitchVelocity());

    // gravity compensation
    pidOutput +=
        PITCH_GRAVITY_COMPENSATION_KP *
        cosf(aruwlib::algorithms::degreesToRadians(turretSubsystem->getPitchAngleFromCenter()));

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

void TurretWorldRelativePositionCommand::end(bool)
{
    turretSubsystem->setYawTarget(
        projectWorldRelativeYawToChassisFrame(yawTargetAngle.getValue(), imuInitialYaw));
}

float TurretWorldRelativePositionCommand::projectChassisRelativeYawToWorldRelative(
    float yawAngle,
    float imuInitialAngle)
{
    return yawAngle + drivers->mpu6500.getYaw() - imuInitialAngle;
}

float TurretWorldRelativePositionCommand::projectWorldRelativeYawToChassisFrame(
    float yawAngle,
    float imuInitialAngle)
{
    return yawAngle - drivers->mpu6500.getYaw() + imuInitialAngle;
}

}  // namespace turret

}  // namespace aruwsrc
