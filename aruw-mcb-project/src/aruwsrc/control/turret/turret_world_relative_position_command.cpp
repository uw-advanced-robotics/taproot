/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include <aruwlib/architecture/clock.hpp>

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace aruwlib::sensors;

namespace aruwsrc
{
namespace turret
{
TurretWorldRelativePositionCommand::TurretWorldRelativePositionCommand(
    aruwlib::Drivers *drivers,
    TurretSubsystem *subsystem,
    const chassis::ChassisSubsystem *chassis,
    bool useImuOnTurret)
    : drivers(drivers),
      turretSubsystem(subsystem),
      chassisSubsystem(chassis),
      yawTargetAngle(TurretSubsystem::TURRET_START_ANGLE, 0.0f, 360.0f),
      currValueImuYawGimbal(0.0f, 0.0f, 360.0f),
      imuInitialYaw(0.0f),
      yawPid(
          YAW_P,
          YAW_I,
          YAW_D_CHASSIS_IMU,
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
      useImuOnTurret(useImuOnTurret)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem *>(subsystem));
}

void TurretWorldRelativePositionCommand::initialize()
{
    imuInitialYaw = drivers->mpu6500.getYaw();
    yawPid.reset();
    pitchPid.reset();
    if (useImuOnTurret && drivers->imuRxHandler.isConnected())
    {
        yawTargetAngle.setValue(drivers->imuRxHandler.getYaw());
        usingImuOnTurret = true;
        yawPid.setD(YAW_D_TURRET_IMU);
    }
    else
    {
        yawTargetAngle.setValue(turretSubsystem->getYawSetpoint());
        usingImuOnTurret = false;
        yawPid.setD(YAW_D_CHASSIS_IMU);
    }
}

void TurretWorldRelativePositionCommand::execute()
{
    turretSubsystem->updateCurrentTurretAngles();
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    float dt = currTime - prevTime;
    prevTime = currTime;
    runYawPositionController(dt);
    runPitchPositionController(dt);
}

void TurretWorldRelativePositionCommand::runYawPositionController(float dt)
{
    // If we are trying to use the IMU on the turret, check to make sure it is available
    // and do some re-initialization if its availability changes
    if (useImuOnTurret)
    {
        bool turretImuOnline = drivers->imuRxHandler.isConnected();
        if (!usingImuOnTurret && turretImuOnline)
        {
            // If we are not using the turret IMU and it has become available, use it
            yawTargetAngle.setValue(drivers->imuRxHandler.getYaw());
            usingImuOnTurret = true;
            yawPid.setD(YAW_D_TURRET_IMU);
        }
        else if (usingImuOnTurret && !turretImuOnline)
        {
            // If the turret IMU has become unavailable, stop using it
            yawTargetAngle.setValue(turretSubsystem->getYawSetpoint());
            usingImuOnTurret = false;
            yawPid.setD(YAW_D_CHASSIS_IMU);
        }
    }

    yawTargetAngle.shiftValue(
        USER_YAW_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretYawInput());

    if (usingImuOnTurret)
    {
        blinkCounter = (blinkCounter + 1) % 100;
        drivers->leds.set(aruwlib::gpio::Leds::Green, blinkCounter > 50);

        float yawActual = drivers->imuRxHandler.getYaw();

        // project target angle from turret imu relative to chassis relative
        turretSubsystem->setYawSetpoint(
            turretSubsystem->getCurrentYawValue().getValue() -
            yawTargetAngle.difference(yawActual));

        if (turretSubsystem->yawLimited())
        {
            // project angle that is limited by subsystem to imu relative again to run the
            // controller
            yawTargetAngle.setValue(
                turretSubsystem->getCurrentYawValue().getValue() -
                turretSubsystem->getYawSetpoint() + yawActual);
        }

        currValueImuYawGimbal.setValue(yawActual);
    }
    else
    {
        // project target angle in world relative to chassis relative to limit the value
        turretSubsystem->setYawSetpoint(
            projectWorldRelativeYawToChassisFrame(yawTargetAngle.getValue(), imuInitialYaw));

        if (turretSubsystem->yawLimited())
        {
            // project angle that is limited by the subsystem to world relative again to run the
            // controller
            yawTargetAngle.setValue(projectChassisRelativeYawToWorldRelative(
                turretSubsystem->getYawSetpoint(),
                imuInitialYaw));
        }

        currValueImuYawGimbal.setValue(projectChassisRelativeYawToWorldRelative(
            turretSubsystem->getCurrentYawValue().getValue(),
            imuInitialYaw));
    }

    // position controller based on imu and yaw gimbal angle
    float positionControllerError = currValueImuYawGimbal.difference(yawTargetAngle);
    float pidOutput;

    if (useImuOnTurret)
    {
        pidOutput =
            yawPid.runController(positionControllerError, drivers->imuRxHandler.getGz(), dt);
    }
    else
    {
        pidOutput = yawPid.runController(
            positionControllerError,
            turretSubsystem->getYawVelocity() + drivers->mpu6500.getGz(),
            dt);
    }

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretWorldRelativePositionCommand::runPitchPositionController(float dt)
{
    // limit the yaw min and max angles
    turretSubsystem->setPitchSetpoint(
        turretSubsystem->getPitchSetpoint() +
        USER_PITCH_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput());

    // position controller based on turret pitch gimbal and imu data
    float positionControllerError =
        turretSubsystem->getCurrentPitchValue().difference(turretSubsystem->getPitchSetpoint());

    float pidOutput =
        pitchPid.runController(positionControllerError, turretSubsystem->getPitchVelocity(), dt);

    // gravity compensation
    pidOutput +=
        PITCH_GRAVITY_COMPENSATION_KP *
        cosf(aruwlib::algorithms::degreesToRadians(turretSubsystem->getPitchAngleFromCenter()));

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

void TurretWorldRelativePositionCommand::end(bool)
{
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
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
