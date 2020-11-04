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

#include "agitator_unjam_command.hpp"

namespace aruwsrc
{
namespace agitator
{
AgitatorUnjamCommand::AgitatorUnjamCommand(
    AgitatorSubsystem* agitator,
    float agitatorMaxUnjamAngle,
    uint32_t agitatorMaxWaitTime)
    : currUnjamstate(AGITATOR_UNJAM_BACK),
      agitatorUnjamRotateTimeout(0),
      salvationTimeout(0),
      agitatorMaxWaitTime(agitatorMaxWaitTime),
      connectedAgitator(agitator),
      agitatorUnjamAngleMax(agitatorMaxUnjamAngle),
      currAgitatorUnjamAngle(0.0f),
      agitatorSetpointBeforeUnjam(0.0f)
{
    if (agitatorMaxUnjamAngle < MIN_AGITATOR_UNJAM_ANGLE)
    {
        agitatorUnjamAngleMax = MIN_AGITATOR_UNJAM_ANGLE;
    }
    this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(agitator));
    salvationTimeout.stop();
    agitatorUnjamRotateTimeout.stop();
}

void AgitatorUnjamCommand::initialize()
{
    agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

    // define a random unjam angle between [MIN_AGITATOR_UNJAM_ANGLE, agitatorUnjamAngleMax]
    float randomUnjamAngle =
        fmodf(rand(), agitatorUnjamAngleMax - MIN_AGITATOR_UNJAM_ANGLE) + MIN_AGITATOR_UNJAM_ANGLE;

    // subtract this angle from the current angle to rotate agitator backwards
    currAgitatorUnjamAngle = connectedAgitator->getAgitatorAngle() - randomUnjamAngle;

    // store the current setpoint angle to be referenced later
    agitatorSetpointBeforeUnjam = connectedAgitator->getAgitatorDesiredAngle();
    currUnjamstate = AGITATOR_UNJAM_BACK;

    salvationTimeout.restart(SALVATION_TIMEOUT_MS);
}

void AgitatorUnjamCommand::execute()
{
    if (salvationTimeout.execute())
    {
        currAgitatorUnjamAngle = agitatorSetpointBeforeUnjam - 2 * aruwlib::algorithms::PI;
        salvationTimeout.stop();
        agitatorUnjamRotateTimeout.restart(SALVATION_UNJAM_BACK_WAIT_TIME);
        currUnjamstate = AGITATOR_SALVATION_UNJAM_BACK;
    }

    switch (currUnjamstate)
    {
        case AGITATOR_SALVATION_UNJAM_BACK:
        {
            connectedAgitator->setAgitatorDesiredAngle(currAgitatorUnjamAngle);
            if (agitatorUnjamRotateTimeout.isExpired() ||
                fabsf(
                    connectedAgitator->getAgitatorAngle() -
                    connectedAgitator->getAgitatorDesiredAngle()) < AGITATOR_SETPOINT_TOLERANCE)
            {
                currUnjamstate = FINISHED;
            }
            break;
        }
        case AGITATOR_UNJAM_BACK:
        {
            connectedAgitator->setAgitatorDesiredAngle(currAgitatorUnjamAngle);
            if (agitatorUnjamRotateTimeout.isExpired() ||
                fabsf(
                    connectedAgitator->getAgitatorAngle() -
                    connectedAgitator->getAgitatorDesiredAngle()) < AGITATOR_SETPOINT_TOLERANCE)
            {  // either the timeout has been triggered or the agitator has reached the setpoint
                // define a random time that the agitator will take to rotate forwards.
                agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

                // reset the agitator
                currUnjamstate = AGITATOR_UNJAM_RESET;
            }
            break;
        }
        case AGITATOR_UNJAM_RESET:  // this is different than just agitator_rotate_command
        {
            // reset the angle to what it was before unjamming
            connectedAgitator->setAgitatorDesiredAngle(agitatorSetpointBeforeUnjam);
            // the agitator is still jammed
            if (agitatorUnjamRotateTimeout.isExpired())
            {
                // restart the timeout
                agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

                // define a new random angle, which will be used in the unjam back state
                float randomUnjamAngle =
                    fmodf(rand(), agitatorUnjamAngleMax - MIN_AGITATOR_UNJAM_ANGLE) +
                    MIN_AGITATOR_UNJAM_ANGLE;

                currAgitatorUnjamAngle = agitatorSetpointBeforeUnjam - randomUnjamAngle;

                currUnjamstate = AGITATOR_UNJAM_BACK;
            }
            else if (
                fabsf(
                    connectedAgitator->getAgitatorAngle() -
                    connectedAgitator->getAgitatorDesiredAngle()) < AGITATOR_SETPOINT_TOLERANCE)
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

void AgitatorUnjamCommand::end(bool) {}

bool AgitatorUnjamCommand::isFinished(void) const { return currUnjamstate == FINISHED; }

}  // namespace agitator

}  // namespace aruwsrc
