/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "unjam_command.hpp"

#include "tap/control/setpoint/interfaces/setpoint_subsystem.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
class SetpointSubsystem;  // forward declaration

UnjamCommand::UnjamCommand(
    SetpointSubsystem* setpointSubsystem,
    float unjamDisplacement,
    float unjamThreshold,
    uint32_t maxWaitTime,
    uint_fast16_t targetCycleCount)
    : unjamRotateTimeout(0),
      maxWaitTime(maxWaitTime),
      setpointSubsystem(setpointSubsystem),
      unjamDisplacement(unjamDisplacement),
      unjamThreshold(unjamThreshold),
      targetCycleCount(targetCycleCount)
{
    unjamDisplacement = abs(unjamDisplacement);
    unjamThreshold = abs(unjamThreshold);
    this->addSubsystemRequirement(setpointSubsystem);
    unjamRotateTimeout.stop();
}

bool UnjamCommand::isReady() { return setpointSubsystem->isOnline(); }

void UnjamCommand::initialize()
{
    unjamRotateTimeout.restart(maxWaitTime);

    // store the current setpoint value to be able to restore subsystem state after command
    // completion
    setpointBeforeUnjam = setpointSubsystem->getSetpoint();

    valueBeforeUnjam = setpointSubsystem->getCurrentValue();

    forwardsCleared = false;
    backwardsCleared = false;
    backwardsCount = 0;

    beginUnjamBackwards();
}

void UnjamCommand::execute()
{
    float currValue = setpointSubsystem->getCurrentValue();

    switch (currUnjamState)
    {
        case UNJAM_BACKWARD:
            if (currValue <= valueBeforeUnjam - unjamThreshold)
            {
                backwardsCleared = true;
                beginUnjamForwards();
            }
            else if (unjamRotateTimeout.isExpired())
            {
                beginUnjamForwards();
            }
            break;
        case UNJAM_FORWARD:
            if (currValue >= valueBeforeUnjam + unjamThreshold)
            {
                forwardsCleared = true;
                beginUnjamBackwards();
            }
            else if (unjamRotateTimeout.isExpired())
            {
                beginUnjamBackwards();
            }
            break;
    }
}

void UnjamCommand::end(bool)
{
    if (forwardsCleared && backwardsCleared)
    {
        setpointSubsystem->clearJam();
        setpointSubsystem->setSetpoint(setpointBeforeUnjam);
    }
    else
    {
        setpointSubsystem->setSetpoint(setpointSubsystem->getCurrentValue());
    }
}

bool UnjamCommand::isFinished() const
{
    return !setpointSubsystem->isOnline() || (forwardsCleared && backwardsCleared) ||
           backwardsCount >= targetCycleCount + 1;
}

void UnjamCommand::beginUnjamForwards()
{
    unjamRotateTimeout.restart(maxWaitTime);
    setpointSubsystem->setSetpoint(valueBeforeUnjam + unjamDisplacement);
    currUnjamState = UNJAM_FORWARD;
}

void UnjamCommand::beginUnjamBackwards()
{
    unjamRotateTimeout.restart(maxWaitTime);
    setpointSubsystem->setSetpoint(valueBeforeUnjam - unjamDisplacement);
    currUnjamState = UNJAM_BACKWARD;
    backwardsCount += 1;
}

}  // namespace setpoint

}  // namespace control

}  // namespace tap
