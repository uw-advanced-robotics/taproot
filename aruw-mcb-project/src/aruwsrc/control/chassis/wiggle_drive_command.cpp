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

#include "aruwlib/algorithms/contiguous_float.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/communication/remote.hpp"
#include "aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "aruwlib/drivers.hpp"

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
      rotationSpeedRamp(0)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
}

void WiggleDriveCommand::initialize()
{
    rotationSign = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;

    // TODO replace with chassis rotation speed
    rotationSpeedRamp.reset(0);
    const WiggleParams& wiggleParams = getWiggleParams();
    rotationSpeedRamp.setTarget(rotationSign * wiggleParams.rotationSpeed);
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
        const float turretYawFromCenter = turret->getYawAngleFromCenter();
        const WiggleParams& wiggleParams = getWiggleParams();

        if (turretYawFromCenter > wiggleParams.turnaroundAngle)
        {
            if (rotationSign > 0)
            {
                rotationSign = -rotationSign;
                rotationSpeedRamp.setTarget(rotationSign * wiggleParams.rotationSpeed);
            }
        }
        else if (turretYawFromCenter < -wiggleParams.turnaroundAngle)
        {
            if (rotationSign < 0)
            {
                rotationSign = -rotationSign;
                rotationSpeedRamp.setTarget(rotationSign * wiggleParams.rotationSpeed);
            }
        }

        rotationSpeedRamp.update(wiggleParams.rotationSpeedIncrement);

        r = rotationSpeedRamp.getValue();
        x *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
        y *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
        // Apply a rotation matrix to the user input so you drive turret
        // relative while wiggling.
        rotateVector(&x, &y, -degreesToRadians(turretYawFromCenter));
    }
    else
    {
        r = drivers->controlOperatorInterface.getChassisRInput() *
            ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    }

    float rTranslationalGain = chassis->calculateRotationTranslationalGain(r) *
                               ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    x = limitVal<float>(x, -rTranslationalGain, rTranslationalGain);
    y = limitVal<float>(y, -rTranslationalGain, rTranslationalGain);

    chassis->setDesiredOutput(x, y, r);
}

void WiggleDriveCommand::end(bool) { chassis->setDesiredOutput(0.0f, 0.0f, 0.0f); }

bool WiggleDriveCommand::isFinished() const { return false; }

const WiggleDriveCommand::WiggleParams& WiggleDriveCommand::getWiggleParams() const
{
    return WIGGLE_PARAMS_45W_CUTOFF;
    uint16_t powerConsumptionLimit =
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    if (powerConsumptionLimit <= 45 || !drivers->refSerial.getRefSerialReceivingData())
    {
        return WIGGLE_PARAMS_45W_CUTOFF;
    }
    else if (powerConsumptionLimit <= 60)
    {
        return WIGGLE_PARAMS_60W_CUTOFF;
    }
    else if (powerConsumptionLimit <= 80)
    {
        return WIGGLE_PARAMS_80W_CUTOFF;
    }
    else
    {
        return WIGGLE_PARAMS_MAX_CUTOFF;
    }
}

}  // namespace chassis

}  // namespace aruwsrc
