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

#include "agitator_shoot_comprised_command_instances.hpp"

namespace aruwsrc
{
namespace agitator
{
ShootFastComprisedCommand17MM::ShootFastComprisedCommand17MM(
    aruwlib::Drivers *drivers,
    AgitatorSubsystem *agitator17mm,
    bool heatLimiting,
    float agitatorRotateAngle)
    : aruwlib::control::setpoint::MoveUnjamComprisedCommand(
          drivers,
          agitator17mm,
          agitatorRotateAngle,
          aruwlib::algorithms::PI / 2.0f,
          40,
          10),
      drivers(drivers),
      heatLimiting(heatLimiting)
{
}

bool ShootFastComprisedCommand17MM::isReady()
{
    const auto &robotData = drivers->refSerial.getRobotData();

    return setpointSubsystem->isOnline() &&
           !(drivers->refSerial.getRefSerialReceivingData() && heatLimiting &&
             (robotData.turret.heat17ID1 + HEAT_LIMIT_BUFFER > robotData.turret.heatLimit17ID1));
}

bool ShootFastComprisedCommand17MM::isFinished() const
{
    const auto &robotData = drivers->refSerial.getRobotData();

    return (!unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&agitatorRotateCommand)) ||
           (unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&agitatorUnjamCommand)) ||
           agitatorDisconnectFault ||
           (drivers->refSerial.getRefSerialReceivingData() && heatLimiting &&
            (robotData.turret.heat17ID1 + HEAT_LIMIT_BUFFER > robotData.turret.heatLimit17ID1));
}

WaterwheelLoadCommand42mm::WaterwheelLoadCommand42mm(
    aruwlib::Drivers *drivers,
    aruwlib::control::setpoint::SetpointSubsystem *waterwheel,
    bool heatLimiting)
    : MoveUnjamComprisedCommand(
          drivers,
          waterwheel,
          WATERWHEEL_42MM_CHANGE_ANGLE,
          WATERWHEEL_42MM_MAX_UNJAM_ANGLE,
          WATERWHEEL_42MM_ROTATE_TIME,
          WATERWHEEL_42MM_PAUSE_AFTER_ROTATE_TIME),
      drivers(drivers),
      waterwheel(waterwheel),
      heatLimiting(heatLimiting)
{
}

bool WaterwheelLoadCommand42mm::isReady()
{
    const auto &robotData = drivers->refSerial.getRobotData();

    return !drivers->refSerial.getRefSerialReceivingData() || !heatLimiting ||
           (robotData.turret.heat42 + HEAT_LIMIT_BUFFER <= robotData.turret.heatLimit42);
}

ShootCommand42mm::ShootCommand42mm(
    aruwlib::Drivers *drivers,
    aruwlib::control::setpoint::SetpointSubsystem *kicker,
    bool heatLimiting)
    : MoveCommand(
          kicker,
          KICKER_42MM_CHANGE_ANGLE,
          KICKER_42MM_ROTATE_TIME,
          KICKER_42MM_PAUSE_AFTER_ROTATE_TIME,
          true),
      drivers(drivers),
      heatLimiting(heatLimiting)
{
}

bool ShootCommand42mm::isReady()
{
    const auto &robotData = drivers->refSerial.getRobotData();

    // (Flywheels have power) &&
    // !(heat limiting data available && apply heat limiting && heat is over limit)
    return robotData.shooterHasPower &&
           !(drivers->refSerial.getRefSerialReceivingData() && heatLimiting &&
             (robotData.turret.heat42 + HEAT_LIMIT_BUFFER > robotData.turret.heatLimit42));
}

}  // namespace agitator

}  // namespace aruwsrc
