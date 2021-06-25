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

#include "sentinel_drive_manual_command.hpp"

#include "aruwlib/Drivers.hpp"
#include "aruwlib/communication/remote.hpp"

#include "sentinel_drive_subsystem.hpp"

using aruwlib::Drivers;
using aruwlib::control::Subsystem;

namespace aruwsrc
{
namespace control
{
SentinelDriveManualCommand::SentinelDriveManualCommand(
    aruwlib::Drivers* drivers,
    SentinelDriveSubsystem* subsystem)
    : Command(),
      drivers(drivers),
      subsystemSentinelDrive(subsystem)
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
}

void SentinelDriveManualCommand::initialize() {}

void SentinelDriveManualCommand::execute()
{
    subsystemSentinelDrive->setDesiredRpm(
        drivers->controlOperatorInterface.getSentinelSpeedInput());
}

void SentinelDriveManualCommand::end(bool) { subsystemSentinelDrive->setDesiredRpm(0); }

bool SentinelDriveManualCommand::isFinished() const { return false; }
}  // namespace control

}  // namespace aruwsrc
