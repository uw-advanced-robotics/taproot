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

#include "move_absolute_command.hpp"

#include "tap/architecture/clock.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
MoveAbsoluteCommand::MoveAbsoluteCommand(
    SetpointSubsystem* setpointSubsystem,
    float setpoint,
    uint32_t speed,
    float setpointTolerance,
    bool automaticallyClearJam)
    : setpointSubsystem(setpointSubsystem),
      setpoint(setpoint),
      speed(speed),
      setpointTolerance(setpointTolerance),
      automaticallyClearJam(automaticallyClearJam)
{
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(setpointSubsystem));
}

void MoveAbsoluteCommand::initialize()
{
    rampTosetpoint.setTarget(setpoint);
    rampTosetpoint.setValue(setpointSubsystem->getCurrentValue());
    prevMoveTime = tap::arch::clock::getTimeMilliseconds();
}

void MoveAbsoluteCommand::execute()
{
    // If the subsystem is jammed, set the setpoint to the current value. Necessary since
    // derived classes may choose to overwrite the `isFinished` function and so for motor safety
    // we do this.
    if (setpointSubsystem->isJammed())
    {
        setpointSubsystem->setSetpoint(setpointSubsystem->getCurrentValue());
        return;
    }

    // We can assume that subsystem is connected, otherwise end will be called.
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    // Divide by 1'000'000 to get setpoint-units because speed is in milli-setpoint-units/second
    // and time interval is in milliseconds. (milliseconds * 1/1000 (second/millisecond) *
    // (milliradians/second) * 1/1000 (units/milli-units) = 1/1'000'000 as our conversion
    rampTosetpoint.update(
        (static_cast<float>(currTime - prevMoveTime) * speed) /
        1'000'000.0f);
    prevMoveTime = currTime;

    setpointSubsystem->setSetpoint(rampTosetpoint.getValue());
}

void MoveAbsoluteCommand::end(bool)
{
    // When this command ends we want to make sure to set the subsystem's target value
    // to it's current value so it doesn't keep trying to move, especially if it's jammed
    // or reached the end of its range of motion.
    setpointSubsystem->setSetpoint(setpointSubsystem->getCurrentValue());
    if (automaticallyClearJam)
    {
        setpointSubsystem->clearJam();
    }
}

bool MoveAbsoluteCommand::isFinished() const
{
    // Command is finished if we've reached target, lost connection to subsystem, or
    // if our subsystem is jammed.
    return (fabsf(setpointSubsystem->getCurrentValue() - rampTosetpoint.getTarget()) <
            setpointTolerance) ||
           !setpointSubsystem->isOnline() || setpointSubsystem->isJammed();
}

}  // namespace setpoint

}  // namespace control

}  // namespace tap
