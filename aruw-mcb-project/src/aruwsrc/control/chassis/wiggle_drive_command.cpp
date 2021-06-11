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

#include "wiggle_drive_command.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/clock.hpp>
#include <aruwlib/communication/remote.hpp>
#include <aruwlib/communication/sensors/mpu6500/mpu6500.hpp>

#include "chassis_subsystem.hpp"

using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;
using aruwlib::Drivers;

namespace aruwsrc
{
namespace chassis
{
WiggleDriveCommand::WiggleDriveCommand(
    aruwlib::Drivers* drivers,
    ChassisSubsystem* chassis,
    aruwsrc::turret::TurretSubsystem* turret)
    : drivers(drivers),
      chassis(chassis),
      turret(turret),
      turretYawRamp(0)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
}

void WiggleDriveCommand::initialize()
{
    float turretCurAngle = turret->getYawAngleFromCenter();

    // We don't apply the sine wave if we are out of the angle of the sine wave.
    outOfCenter = fabsf(turretCurAngle) > WIGGLE_MAX_ROTATE_ANGLE;

    turretCurAngle = aruwlib::algorithms::limitVal(
        turretCurAngle,
        -WIGGLE_MAX_ROTATE_ANGLE,
        WIGGLE_MAX_ROTATE_ANGLE);

    // We use the limited current turret angle to calculate a time offset in
    // a angle vs. time graph so when we start at some angle from center we
    // are still in phase.
    startTimeForAngleOffset = asinf(turretCurAngle / WIGGLE_MAX_ROTATE_ANGLE) * getPeriod() /
                              (2.0f * aruwlib::algorithms::PI);

    // The offset so when we start calculating a rotation angle, the initial
    // time is zero.
    timeOffset = aruwlib::arch::clock::getTimeMilliseconds();

    float turretYaw = turret->getYawAngleFromCenter();
    turretYawRamp.setTarget(turretYaw);
    turretYawRamp.setValue(turretYaw);
}

float WiggleDriveCommand::wiggleSin(float t)
{
    return WIGGLE_MAX_ROTATE_ANGLE * sinf((2.0f * aruwlib::algorithms::PI / getPeriod()) * t);
}

void WiggleDriveCommand::execute()
{
    float r;
    float x = drivers->controlOperatorInterface.getChassisXInput() *
              ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    float y = drivers->controlOperatorInterface.getChassisYInput() *
              ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    // We only wiggle when the turret is online.
    if (turret->isOnline())
    {
        float curTime =
            static_cast<float>(aruwlib::arch::clock::getTimeMilliseconds() - timeOffset) -
            startTimeForAngleOffset;
        float desiredAngleError;

        float turretYaw = turret->getYawAngleFromCenter();
        turretYawRamp.setTarget(turretYaw);
        turretYawRamp.update(TURRET_YAW_TARGET_RAMP_INCREMENT);
        float turretYawFiltered = turretYawRamp.getValue();

        if (outOfCenter)
        {
            outOfCenter = fabsf(turretYaw) > WIGGLE_MAX_ROTATE_ANGLE;
            if (!outOfCenter)
            {
                initialize();
            }
        }

        if (!outOfCenter)
        {
            desiredAngleError = wiggleSin(curTime) + turretYawFiltered;
        }
        else
        {
            desiredAngleError = aruwlib::algorithms::limitVal(
                turretYawFiltered,
                -WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR,
                WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR);
        }

        // Wrapping between -180 and 180.
        ContiguousFloat rotationError(desiredAngleError, -180.0f, 180.0f);
        r = chassis->chassisSpeedRotationPID(rotationError.getValue(), WIGGLE_ROTATE_KP);
        x *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
        y *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
        // Apply a rotation matrix to the user input so you drive turret
        // relative while wiggling.
        aruwlib::algorithms::rotateVector(&x, &y, -degreesToRadians(turretYaw));
    }
    else
    {
        r = drivers->controlOperatorInterface.getChassisRInput() *
            ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    }

    float rTranslationalGain = chassis->calculateRotationTranslationalGain(r) *
                               ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    x = aruwlib::algorithms::limitVal<float>(x, -rTranslationalGain, rTranslationalGain);
    y = aruwlib::algorithms::limitVal<float>(y, -rTranslationalGain, rTranslationalGain);

    chassis->setDesiredOutput(x, y, r);
}

void WiggleDriveCommand::end(bool) { chassis->setDesiredOutput(0.0f, 0.0f, 0.0f); }

bool WiggleDriveCommand::isFinished() const { return false; }

float WiggleDriveCommand::getPeriod() const
{
    uint16_t powerConsumptionLimit =
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    if (powerConsumptionLimit <= 45 || !drivers->refSerial.getRefSerialReceivingData())
    {
        return WIGGLE_PERIOD_45W_CUTOFF;
    }
    else if (powerConsumptionLimit <= 60)
    {
        return WIGGLE_PERIOD_60W_CUTOFF;
    }
    else if (powerConsumptionLimit <= 80)
    {
        return WIGGLE_PERIOD_80W_CUTOFF;
    }
    else
    {
        return WIGGLE_PERIOD_MAX_CUTOFF;
    }
}

}  // namespace chassis

}  // namespace aruwsrc
