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

#ifndef PLATFORM_HOSTED
#include <modm/platform/random/random_number_generator.hpp>
#endif

#include "sentinel_auto_drive_command.hpp"
#include "sentinel_drive_subsystem.hpp"

#ifndef PLATFORM_HOSTED
using modm::platform::RandomNumberGenerator;
#endif
using aruwlib::control::Subsystem;

namespace aruwsrc
{
namespace control
{
SentinelAutoDriveCommand::SentinelAutoDriveCommand(SentinelDriveSubsystem* subsystem)
    : subsystemSentinelDrive(subsystem),
      changeVelocityTimer(CHANGE_TIME_INTERVAL)
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
#ifndef PLATFORM_HOSTED
    RandomNumberGenerator::enable();
#endif
}

void SentinelAutoDriveCommand::initialize() { chosenNewRPM = false; }

void SentinelAutoDriveCommand::execute()
{
    if (this->changeVelocityTimer.isExpired() || !chosenNewRPM)
    {
#ifdef PLATFORM_HOSTED
        chosenNewRPM = true;
#else
        chosenNewRPM = RandomNumberGenerator::isReady();
#endif
        if (chosenNewRPM)
        {
            this->changeVelocityTimer.restart(CHANGE_TIME_INTERVAL);
#ifdef PLATFORM_HOSTED
            currentRPM = MIN_RPM + (MAX_RPM - MIN_RPM) / 2;
#else
            uint32_t randVal = RandomNumberGenerator::getValue();
            currentRPM = randVal % (MAX_RPM - MIN_RPM + 1) + MIN_RPM;
            if (randVal % 2 == 0)
            {
                currentRPM *= -1.0f;
            }
#endif
        }
    }

    // reverse direction if close to the end of the rail
    float curPos = subsystemSentinelDrive->absolutePosition();
    if ((currentRPM < 0 && curPos < RAIL_BUFFER) ||
        (currentRPM > 0 && curPos > SentinelDriveSubsystem::RAIL_LENGTH - RAIL_BUFFER))
    {
        currentRPM = -currentRPM;
    }

    subsystemSentinelDrive->setDesiredRpm(currentRPM);
}

void SentinelAutoDriveCommand::end(bool) { subsystemSentinelDrive->setDesiredRpm(0); }

bool SentinelAutoDriveCommand::isFinished() const { return false; }
}  // namespace control

}  // namespace aruwsrc
