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

#include "turret_init_command.hpp"

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/control/command.hpp>

#include "turret_subsystem.hpp"

namespace aruwsrc
{
namespace turret
{
TurretInitCommand::TurretInitCommand(TurretSubsystem *subsystem)
    : turretSubsystem(subsystem),
      initYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
      initPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

bool TurretInitCommand::isFinished() const
{
    return fabsf(turretSubsystem->getPitchAngleFromCenter()) < 5.0f &&
           fabsf(turretSubsystem->getYawAngleFromCenter()) < 5.0f &&
           turretSubsystem->isTurretOnline();
}

void TurretInitCommand::end(bool) {}

void TurretInitCommand::execute() { updateTurretPosition(); }

void TurretInitCommand::updateTurretPosition()
{
    initPitchPid.update(turretSubsystem->getPitchAngle().difference(pitchTargetAngle));
    initYawPid.update(turretSubsystem->getYawAngle().difference(yawTargetAngle));
    turretSubsystem->setPitchMotorOutput(initPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(initYawPid.getValue());
}

}  // namespace turret

}  // namespace aruwsrc
