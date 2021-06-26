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

#ifndef SENTINEL_AGITATOR_SYSTEM_COMPRISED_COMMAND_HPP_
#define SENTINEL_AGITATOR_SYSTEM_COMPRISED_COMMAND_HPP_

#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/control/comprised_command.hpp"
#include "aruwlib/control/setpoint/commands/move_unjam_comprised_command.hpp"

namespace aruwsrc::agitator
{
class AgitatorSubsystem;
}

namespace aruwsrc::control::sentinel::firing
{
class SentinelSwitcherSubsystem;

/**
 * A command that rotates an agitator that feeds into two barrels. A switcher is
 * used to alternate between barrels based on the heat limit of the barrel.
 */
class SentinelRotateAgitatorCommand : public aruwlib::control::ComprisedCommand
{
public:
    SentinelRotateAgitatorCommand(
        aruwlib::Drivers* drivers,
        aruwlib::control::setpoint::SetpointSubsystem* agitator,
        SentinelSwitcherSubsystem* switcher);

    const char* getName() const override { return "sentinel rotate agitator"; }

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    static constexpr uint32_t SWITCH_BARREL_TIMEOUT = 100;
    static constexpr uint16_t BARREL_OVERHEAT_THRESHOLD = 30;
    static constexpr float AGITATOR_ROTATE_ANGLE = 3 * aruwlib::algorithms::PI / 10;
    static constexpr float AGITATOR_ROTATE_MAX_UNJAM_ANGLE = aruwlib::algorithms::PI / 4;
    static constexpr uint32_t AGITATOR_ROTATE_TIME = 54;
    static constexpr uint32_t AGITATOR_WAIT_AFTER_ROTATE_TIME = 0;

    aruwlib::Drivers* drivers;
    aruwlib::control::setpoint::SetpointSubsystem* agitator;
    SentinelSwitcherSubsystem* switcher;

    aruwlib::control::setpoint::MoveUnjamComprisedCommand rotateAgitator;

    aruwlib::arch::MilliTimeout switchBarrelTimeout;

    bool switchingBarrel = false;
};

}  // namespace aruwsrc::control::sentinel::firing

#endif  // SENTINEL_AGITATOR_SYSTEM_COMPRISED_COMMAND_HPP_
