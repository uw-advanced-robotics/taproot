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
#ifndef SENTINEL_AUTO_DRIVE_COMPRISED_COMMAND_HPP_
#define SENTINEL_AUTO_DRIVE_COMPRISED_COMMAND_HPP_

#include "aruwlib/control/comprised_command.hpp"

#include "sentinel_full_traverse_command.hpp"
#include "sentinel_random_drive_command.hpp"

namespace aruwsrc::control
{
class SentinelDriveSubsystem;
/**
 * A command that alternates between base sentinel drive commands depending on the
 * damage it is currently engaging and **eventually** based on which targets are being
 * engaged
 */
class SentinelAutoDriveComprisedCommand : public aruwlib::control::ComprisedCommand
{
public:
    SentinelAutoDriveComprisedCommand(
        aruwlib::Drivers *drivers,
        SentinelDriveSubsystem *sentinelChassis);

    const char *getName() const override { return "sentinel random drive"; }
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    static constexpr float RANDOM_DRIVE_DPS_THRESHOLD = 5;
    aruwlib::Drivers *drivers;
    SentinelDriveSubsystem *sentinelChassis;
    SentinelFullTraverseCommand fullTraverse;
    SentinelRandomDriveCommand randomDrive;
    bool evadeMode;
};
}  // namespace aruwsrc::control

#endif  // SENTINEL_AUTO_DRIVE_COMPRISED_COMMAND_HPP_
#endif
