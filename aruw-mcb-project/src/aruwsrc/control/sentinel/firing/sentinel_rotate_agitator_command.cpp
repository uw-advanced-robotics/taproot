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

#include "sentinel_rotate_agitator_command.hpp"

#include "aruwlib/drivers.hpp"

#include "sentinel_switcher_subsystem.hpp"

using namespace aruwlib::control;

#define TURRET_OVERHEAT(currHeat, heatLimit) (currHeat + BARREL_OVERHEAT_THRESHOLD > heatLimit)

namespace aruwsrc::control::sentinel::firing
{
SentinelRotateAgitatorCommand::SentinelRotateAgitatorCommand(
    aruwlib::Drivers* drivers,
    aruwlib::control::setpoint::SetpointSubsystem* agitator,
    SentinelSwitcherSubsystem* switcher)
    : ComprisedCommand(drivers),
      drivers(drivers),
      agitator(agitator),
      switcher(switcher),
      rotateAgitator(
          drivers,
          agitator,
          AGITATOR_ROTATE_ANGLE,
          AGITATOR_ROTATE_MAX_UNJAM_ANGLE,
          AGITATOR_ROTATE_TIME,
          AGITATOR_WAIT_AFTER_ROTATE_TIME)
{
    addSubsystemRequirement(agitator);
    addSubsystemRequirement(switcher);
    comprisedCommandScheduler.registerSubsystem(agitator);
}

bool SentinelRotateAgitatorCommand::isReady()
{
    if (!drivers->refSerial.getRefSerialReceivingData())
    {
        return true;
    }

    auto turret = drivers->refSerial.getRobotData().turret;
    return (!TURRET_OVERHEAT(turret.heat17ID1, turret.heatLimit17ID1) ||
            !TURRET_OVERHEAT(turret.heat17ID2, turret.heatLimit17ID2)) &&
           rotateAgitator.isReady();
}

void SentinelRotateAgitatorCommand::initialize()
{
    // Switch barrel if necessary
    auto turret = drivers->refSerial.getRobotData().turret;
    bool lowerUsed = switcher->isLowerUsed();
    if (drivers->refSerial.getRefSerialReceivingData() &&
        ((lowerUsed && TURRET_OVERHEAT(turret.heat17ID1, turret.heatLimit17ID1)) ||
         (!lowerUsed && TURRET_OVERHEAT(turret.heat17ID2, turret.heatLimit17ID2))))
    {
        switcher->useLowerBarrel(!lowerUsed);
        switchBarrelTimeout.restart(SWITCH_BARREL_TIMEOUT);
        switchingBarrel = true;
    }
    else
    {
        switchBarrelTimeout.stop();
        comprisedCommandScheduler.addCommand(&rotateAgitator);
    }
}

void SentinelRotateAgitatorCommand::execute()
{
    if (switchBarrelTimeout.execute())
    {
        // Done switching barrel, start rotating agitator
        switchingBarrel = false;
        comprisedCommandScheduler.addCommand(&rotateAgitator);
    }
    comprisedCommandScheduler.run();
}

void SentinelRotateAgitatorCommand::end(bool interrupted)
{
    switchingBarrel = false;
    comprisedCommandScheduler.removeCommand(&rotateAgitator, interrupted);
}

bool SentinelRotateAgitatorCommand::isFinished() const
{
    return !switchingBarrel && rotateAgitator.isFinished();
}

}  // namespace aruwsrc::control::sentinel::firing
