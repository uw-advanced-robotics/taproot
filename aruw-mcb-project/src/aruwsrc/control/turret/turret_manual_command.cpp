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

#include "turret_manual_command.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/communication/remote.hpp>

#include "turret_subsystem.hpp"

using aruwlib::Drivers;

namespace aruwsrc
{
namespace turret
{
TurretManualCommand::TurretManualCommand(aruwlib::Drivers *drivers, TurretSubsystem *subsystem)
    : drivers(drivers),
      turretSubsystem(subsystem),
      manualYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
      manualPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

bool TurretManualCommand::isFinished() const { return false; }

void TurretManualCommand::execute()
{
    pitchVelocityTarget =
        USER_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput();
    yawVelocityTarget = USER_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretYawInput();

    manualPitchPid.update(pitchVelocityTarget - turretSubsystem->getPitchVelocity());
    manualYawPid.update(yawVelocityTarget - turretSubsystem->getYawVelocity());

    turretSubsystem->setPitchMotorOutput(manualPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(manualYawPid.getValue());
}

}  // namespace turret

}  // namespace aruwsrc
