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
    float maximumDisplacement,
    float unjamDisplacement,
    uint32_t maxWaitTime)
    : currUnjamstate(UNJAM_BACK),
      unjamRotateTimeout(0),
      salvationTimeout(0),
      maxWaitTime(maxWaitTime),
      setpointSubsystem(setpointSubsystem),
      maxUnjamDisplacement(maximumDisplacement),
      unjamDisplacement(unjamDisplacement),
      currUnjamDisplacement(0.0f),
      setpointBeforeUnjam(0.0f)
{
    if (maximumDisplacement < MIN_UNJAM_DISPLACEMENT)
    {
        maxUnjamDisplacement = MIN_UNJAM_DISPLACEMENT;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(setpointSubsystem));
    salvationTimeout.stop();
    unjamRotateTimeout.stop();
}

void UnjamCommand::initialize()
{
    unjamRotateTimeout.restart(maxWaitTime);

    // define a random unjam displacement between [MIN_UNJAM_DISPLACEMENT, maxUnjamDisplacement]
    const float minUnjamAngle =
        maxUnjamDisplacement <= MIN_UNJAM_DISPLACEMENT ? 0 : MIN_UNJAM_DISPLACEMENT;
    float randomUnjamAngle = fmodf(rand(), maxUnjamDisplacement - minUnjamAngle) + minUnjamAngle;

    // subtract this value from the current value to move subsystem backwards
    currUnjamDisplacement = setpointSubsystem->getCurrentValue() - randomUnjamAngle;

    // store the current setpoint value to be referenced later
    setpointBeforeUnjam = setpointSubsystem->getSetpoint();
    currUnjamstate = UNJAM_BACK;

    salvationTimeout.restart(SALVATION_TIMEOUT_MS);
}

void UnjamCommand::execute()
{
    // Don't run logic if subsystem offline
    if (!setpointSubsystem->isOnline())
    {
        return;
    }
    if (salvationTimeout.execute())
    {
        currUnjamDisplacement = setpointBeforeUnjam - 2 * tap::algorithms::PI;
        salvationTimeout.stop();
        unjamRotateTimeout.restart(SALVATION_UNJAM_BACK_WAIT_TIME);
        currUnjamstate = SALVATION_UNJAM_BACK;
    }

    switch (currUnjamstate)
    {
        case SALVATION_UNJAM_BACK:
        {
            setpointSubsystem->setSetpoint(currUnjamDisplacement);
            if (unjamRotateTimeout.isExpired() ||
                fabsf(setpointSubsystem->getCurrentValue() - setpointSubsystem->getSetpoint()) <
                    SETPOINT_TOLERANCE)
            {
                currUnjamstate = FINISHED;
            }
            break;
        }
        case UNJAM_BACK:
        {
            setpointSubsystem->setSetpoint(currUnjamDisplacement);
            if (unjamRotateTimeout.isExpired() ||
                fabsf(setpointSubsystem->getCurrentValue() - setpointSubsystem->getSetpoint()) <
                    SETPOINT_TOLERANCE)
            {  // either the timeout has been triggered or the subsystem has reached the setpoint
                // define a random time that the subsystem will take to rotate forwards.
                unjamRotateTimeout.restart(maxWaitTime);

                // reset the subsystem
                currUnjamstate = UNJAM_RESET;
            }
            break;
        }
        case UNJAM_RESET:  // this is different than just move_command
        {
            // reset the value to what it was before unjamming
            setpointSubsystem->setSetpoint(setpointBeforeUnjam);
            // the subsystem is still jammed
            if (unjamRotateTimeout.isExpired())
            {
                // restart the timeout
                unjamRotateTimeout.restart(maxWaitTime);

                // define a new random value, which will be used in the unjam back state
                const float minUnjamAngle = maxUnjamDisplacement <= MIN_UNJAM_DISPLACEMENT
                                                ? 0
                                                : MIN_UNJAM_DISPLACEMENT;
                float randomUnjamAngle =
                    fmodf(rand(), maxUnjamDisplacement - minUnjamAngle) + minUnjamAngle;

                currUnjamDisplacement = setpointBeforeUnjam - randomUnjamAngle;

                currUnjamstate = UNJAM_BACK;
            }
            else if (
                fabsf(setpointSubsystem->getCurrentValue() - setpointSubsystem->getSetpoint()) <
                SETPOINT_TOLERANCE)
            {
                currUnjamstate = FINISHED;
            }
            break;
        }
        case FINISHED:  // this could be only two states, but its simpler to debug with three
        {
            break;
        }
    }
}

void UnjamCommand::end(bool) { setpointSubsystem->clearJam(); }

bool UnjamCommand::isFinished(void) const { return currUnjamstate == FINISHED; }

}  // namespace setpoint

}  // namespace control

}  // namespace tap
