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

#include "beyblade_command.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/communication/remote.hpp"
#include "aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "aruwlib/drivers.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_subsystem.hpp"

using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;
using aruwlib::Drivers;

namespace aruwsrc
{
namespace chassis
{
BeybladeCommand::BeybladeCommand(
    aruwlib::Drivers* drivers,
    ChassisSubsystem* chassis,
    const aruwlib::control::turret::TurretSubsystemInterface* turret)
    : drivers(drivers),
      chassis(chassis),
      turret(turret)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
}

// Resets ramp
void BeybladeCommand::initialize()
{
#ifdef ENV_UNIT_TESTS
    rotationDirection = 1;
#else
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;
#endif
    rotateSpeedRamp.reset(chassis->getDesiredRotation());
}

void BeybladeCommand::execute()
{
    // Gets current turret yaw angle
    float turretYawAngle = turret->getYawAngleFromCenter();

    // Get X and Y speed inputs with translational movement
    float x = drivers->controlOperatorInterface.getChassisXInput() *
              ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR *
              TRANSLATIONAL_SPEED_FRACTION_WHILE_BEYBLADE;
    float y = drivers->controlOperatorInterface.getChassisYInput() *
              ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR *
              TRANSLATIONAL_SPEED_FRACTION_WHILE_BEYBLADE;

    static constexpr float TRANSLATION_LIMIT = TRANSLATION_LIMITING_FRACTION *
                                               TRANSLATIONAL_SPEED_FRACTION_WHILE_BEYBLADE *
                                               ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    rampTarget = rotationDirection * getRotationTarget();
    if (x > TRANSLATION_LIMIT || y > TRANSLATION_LIMIT)
    {
        rampTarget *= RAMP_TARGET_TRANSLATIONAL_FRAC;
    }

    rotateSpeedRamp.setTarget(rampTarget);
    // Update the r speed by 1/8 of target (linear for each update)
    rotateSpeedRamp.update(rampTarget * RAMP_UPDATE_FRAC);
    float r = rotateSpeedRamp.getValue();

    // Rotate X and Y depending on turret angle
    aruwlib::algorithms::rotateVector(&x, &y, -degreesToRadians(turretYawAngle));

    // set outputs
    chassis->setDesiredOutput(x, y, r);
}

void BeybladeCommand::end(bool) { chassis->setDesiredOutput(0.0f, 0.0f, 0.0f); }

bool BeybladeCommand::isFinished() const { return false; }

float BeybladeCommand::getRotationTarget() const
{
    const uint16_t powerConsumptionLimit =
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    if (!drivers->refSerial.getRefSerialReceivingData() || powerConsumptionLimit <= 45)
    {
        return ROTATION_TARGET_45W_CUTOFF;
    }
    else if (powerConsumptionLimit <= 60)
    {
        return ROTATION_TARGET_60W_CUTOFF;
    }
    else if (powerConsumptionLimit <= 80)
    {
        return ROTATION_TARGET_80W_CUTOFF;
    }
    else
    {
        return ROTATION_TARGET_MAX_CUTOFF;
    }
}
}  // namespace chassis

}  // namespace aruwsrc
