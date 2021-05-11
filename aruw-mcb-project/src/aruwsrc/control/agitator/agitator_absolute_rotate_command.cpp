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

#include "agitator_absolute_rotate_command.hpp"

#include <aruwlib/architecture/clock.hpp>

using aruwsrc::agitator::AgitatorSubsystem;

namespace aruwsrc
{
namespace control
{
AgitatorAbsoluteRotateCommand::AgitatorAbsoluteRotateCommand(
    AgitatorSubsystem* agitator,
    float targetAngle,
    uint32_t agitatorRotateSpeed,
    float setpointTolerance)
    : connectedAgitator(agitator),
      targetAngle(targetAngle),
      agitatorRotateSpeed(agitatorRotateSpeed),
      agitatorSetpointTolerance(setpointTolerance)
{
    this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(agitator));
}

void AgitatorAbsoluteRotateCommand::initialize()
{
    rampToTargetAngle.setTarget(targetAngle);
    rampToTargetAngle.setValue(connectedAgitator->getAgitatorAngle());
    agitatorPrevRotateTime = aruwlib::arch::clock::getTimeMilliseconds();
    jamTimeout.restart(AGITATOR_JAM_TIMEOUT);
}

void AgitatorAbsoluteRotateCommand::execute()
{
    // Reset jam timeout if agitator is within setpoint tolerance of where target from
    // last command execution
    float desiredAngle = rampToTargetAngle.getValue();
    float currAngle = connectedAgitator->getAgitatorAngle();
    if (fabsf(currAngle - desiredAngle) < AGITATOR_SETPOINT_TOLERANCE)
    {
        jamTimeout.restart(AGITATOR_JAM_TIMEOUT);
    }

    // We can assume that agitator is connected, otherwise end will be called.
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    // Divide by 1'000'000 to get radians because agitatorRotateSpeed is in milliRadians/second
    // and time interval is in milliseconds. (milliseconds * 1/1000 (second/millisecond) *
    // (milliradians/second) * 1/1000 (radians/milliradian) = 1/1'000'000 as our conversion
    rampToTargetAngle.update(
        (static_cast<float>(currTime - agitatorPrevRotateTime) * agitatorRotateSpeed) /
        1'000'000.0f);
    agitatorPrevRotateTime = currTime;

    connectedAgitator->setAgitatorDesiredAngle(rampToTargetAngle.getValue());
}

void AgitatorAbsoluteRotateCommand::end(bool)
{
    // When this command ends we want to make sure to set the agitator's target angle
    // to it's current angle so it doesn't keep trying to move, especially if it's jammed
    // or reached the end of its range of motion.
    connectedAgitator->setAgitatorDesiredAngle(connectedAgitator->getAgitatorAngle());
}

bool AgitatorAbsoluteRotateCommand::isFinished() const
{
    // Command is finished if we've reached target, lost connection to agitator, or
    // if our agitator is jammed.
    return (fabsf(
                connectedAgitator->getAgitatorAngle() -
                connectedAgitator->getAgitatorDesiredAngle()) < agitatorSetpointTolerance) ||
           !connectedAgitator->isAgitatorOnline() || jamTimeout.isExpired();
}

}  // namespace control

}  // namespace aruwsrc
