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

#if defined(TARGET_SENTINEL)

#include "sentinel_full_traverse_command.hpp"

using namespace aruwlib::arch::clock;

namespace aruwsrc::control::sentinel::drive
{
SentinelFullTraverseCommand::SentinelFullTraverseCommand(SentinelDriveSubsystem* subsystem)
    : prevTime(0),
      velocityTargetGenerator(0),
      subsystemSentinelDrive(subsystem)
{
    addSubsystemRequirement(subsystem);
}

void SentinelFullTraverseCommand::initialize()
{
    prevTime = getTimeMilliseconds();
    velocityTargetGenerator.reset(0);
    velocityTargetGenerator.setTarget(MAX_DESIRED_TRAVERSE_SPEED);
}

void SentinelFullTraverseCommand::execute()
{
    float curPos = subsystemSentinelDrive->absolutePosition();
    // reverse direction if close to the end of the rail
    if (velocityTargetGenerator.getValue() < 0 && curPos < TURNAROUND_BUFFER)
    {
        velocityTargetGenerator.setTarget(MAX_DESIRED_TRAVERSE_SPEED);
    }
    else if (
        velocityTargetGenerator.getValue() > 0 &&
        curPos > SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH -
                     TURNAROUND_BUFFER)
    {
        velocityTargetGenerator.setTarget(-MAX_DESIRED_TRAVERSE_SPEED);
    }
    // update chassis target velocity
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    velocityTargetGenerator.update(RAMP_SPEED * static_cast<float>(currTime - prevTime));
    prevTime = currTime;
    subsystemSentinelDrive->setDesiredRpm(velocityTargetGenerator.getValue());
}

void SentinelFullTraverseCommand::end(bool) { subsystemSentinelDrive->setDesiredRpm(0.0f); }

bool SentinelFullTraverseCommand::isFinished() const { return false; }
}  // namespace aruwsrc::control::sentinel::drive

#endif
