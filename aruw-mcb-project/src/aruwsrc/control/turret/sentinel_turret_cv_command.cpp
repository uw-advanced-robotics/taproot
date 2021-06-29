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

#include "sentinel_turret_cv_command.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/control/comprised_command.hpp"
#include "aruwlib/drivers.hpp"

#include "aruwsrc/communication/serial/xavier_serial.hpp"
#include "aruwsrc/control/turret/double_pitch_turret_subsystem.hpp"

using namespace aruwlib;
using namespace aruwlib::algorithms;

namespace aruwsrc::control::turret
{
SentinelTurretCVCommand::SentinelTurretCVCommand(
    aruwlib::Drivers *drivers,
    aruwlib::control::turret::TurretSubsystemInterface *sentinelTurret,
    aruwsrc::agitator::AgitatorSubsystem *agitator,
    sentinel::firing::SentinelSwitcherSubsystem *switcher)
    : aruwlib::control::ComprisedCommand(drivers),
      drivers(drivers),
      sentinelTurret(sentinelTurret),
      rotateAgitator(drivers, agitator, switcher),
      aimingAtTarget(false),
      lostTargetCounter(0)
{
    addSubsystemRequirement(agitator);
    addSubsystemRequirement(sentinelTurret);
    addSubsystemRequirement(switcher);
    comprisedCommandScheduler.registerSubsystem(agitator);
    comprisedCommandScheduler.registerSubsystem(sentinelTurret);
    comprisedCommandScheduler.registerSubsystem(switcher);
}

void SentinelTurretCVCommand::initialize()
{
    drivers->xavierSerial.beginAutoAim();
    pitchScanningUp = false;
    yawScanningRight = false;
    lostTargetCounter = 0;
}

void SentinelTurretCVCommand::execute()
{
    if (drivers->xavierSerial.lastAimDataValid())
    {
        const auto &cvData = drivers->xavierSerial.getLastAimData();
        if (cvData.hasTarget)
        {
            aimingAtTarget = true;
            sentinelTurret->setYawSetpoint(cvData.yaw);
            sentinelTurret->setPitchSetpoint(cvData.pitch);

            if (fabs(sentinelTurret->getCurrentYawValue().difference(cvData.yaw)) <=
                    YAW_FIRE_ERROR_MARGIN &&
                fabs(sentinelTurret->getCurrentPitchValue().difference(cvData.pitch)) <=
                    PITCH_FIRE_ERROR_MARGIN)
            {
                if (!comprisedCommandScheduler.isCommandScheduled(&rotateAgitator))
                {
                    comprisedCommandScheduler.addCommand(&rotateAgitator);
                }
            }
        }
        else
        {
            scanForTarget();
        }
    }
    else
    {
        scanForTarget();
    }

    comprisedCommandScheduler.run();
}

void SentinelTurretCVCommand::end(bool interrupted)
{
    drivers->xavierSerial.stopAutoAim();
    comprisedCommandScheduler.removeCommand(&rotateAgitator, interrupted);
}

void SentinelTurretCVCommand::updateScanningUp(
    const float motorSetpoint,
    const float minMotorSetpoint,
    const float maxMotorSetpoint,
    bool *axisScanningUp)
{
    if (motorSetpoint > maxMotorSetpoint)
    {
        *axisScanningUp = false;
    }
    else if (motorSetpoint < minMotorSetpoint)
    {
        *axisScanningUp = true;
    }
}

void SentinelTurretCVCommand::scanForTarget()
{
    // Increment aim counter and stop aiming at target if the target has been lost for some time
    if (aimingAtTarget)
    {
        lostTargetCounter++;

        if (lostTargetCounter > AIM_LOST_NUM_COUNTS)
        {
            lostTargetCounter = 0;
            aimingAtTarget = false;
        }
        else
        {
            return;
        }
    }

    const float pitchSetpoint = sentinelTurret->getPitchSetpoint();

    updateScanningUp(
        pitchSetpoint,
        DoublePitchTurretSubsystem::TURRET_PITCH_MIN_ANGLE,
        DoublePitchTurretSubsystem::TURRET_PITCH_MAX_ANGLE,
        &pitchScanningUp);

    sentinelTurret->setPitchSetpoint(
        pitchSetpoint + (pitchScanningUp ? SCAN_DELTA_ANGLE_PITCH : -SCAN_DELTA_ANGLE_PITCH));

    const float yawSetpoint = sentinelTurret->getYawSetpoint();

    updateScanningUp(
        yawSetpoint,
        DoublePitchTurretSubsystem::TURRET_YAW_MIN_ANGLE,
        DoublePitchTurretSubsystem::TURRET_YAW_MAX_ANGLE,
        &yawScanningRight);

    sentinelTurret->setYawSetpoint(
        yawSetpoint + (yawScanningRight ? SCAN_DELTA_ANGLE_YAW : -SCAN_DELTA_ANGLE_YAW));
}

}  // namespace aruwsrc::control::turret
