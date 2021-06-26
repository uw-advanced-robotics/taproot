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

#ifndef SENTINEL_DRIVE_MANUAL_COMMAND_HPP_
#define SENTINEL_DRIVE_MANUAL_COMMAND_HPP_

#include "aruwlib/control/command.hpp"

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc::control::sentinel::drive
{
class SentinelDriveSubsystem;

class SentinelDriveManualCommand : public aruwlib::control::Command
{
public:
    SentinelDriveManualCommand(aruwlib::Drivers* drivers, SentinelDriveSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "sentinel drive manual"; }

private:
    aruwlib::Drivers* drivers;

    SentinelDriveSubsystem* subsystemSentinelDrive;
};

}  // namespace aruwsrc::control::sentinel::drive

#endif
