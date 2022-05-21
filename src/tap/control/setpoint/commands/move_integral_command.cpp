/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "move_integral_command.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

using namespace tap::algorithms;

namespace tap::control::setpoint
{
MoveIntegralCommand::MoveIntegralCommand(
    IntegrableSetpointSubsystem& integrableSetpointSubsystem,
    const Config& config)
    : config(config),
      integrableSetpointSubsystem(integrableSetpointSubsystem)
{
    assert(config.integralSetpointTolerance >= 0);
    assert(getSign(config.targetIntegralChange) == getSign(config.desiredSetpoint));

    addSubsystemRequirement(&integrableSetpointSubsystem);
}

void MoveIntegralCommand::initialize()
{
    integrableSetpointSubsystem.setSetpoint(config.desiredSetpoint);
    integrableSetpointSubsystem.setDesiredIntegralSetpoint(
        integrableSetpointSubsystem.getDesiredIntegralSetpoint() + config.targetIntegralChange);
}

void MoveIntegralCommand::end(bool) { integrableSetpointSubsystem.setSetpoint(0); }

bool MoveIntegralCommand::isFinished() const
{
    // The subsystem is jammed or offline or it is within the setpoint tolerance
    return integrableSetpointSubsystem.isJammed() || !integrableSetpointSubsystem.isOnline() ||
           targetIntegralReached();
}
}  // namespace tap::control::setpoint
