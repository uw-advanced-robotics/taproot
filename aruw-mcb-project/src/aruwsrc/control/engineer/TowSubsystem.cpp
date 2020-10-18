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

#include "TowSubsystem.hpp"

#include <aruwlib/Drivers.hpp>

namespace aruwsrc
{
namespace engineer
{
TowSubsystem::TowSubsystem(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Digital::OutputPin leftTowPin,
    aruwlib::gpio::Digital::OutputPin rightTowPin,
    aruwlib::gpio::Digital::InputPin leftTowLimitSwitchPin,
    aruwlib::gpio::Digital::InputPin rightTowLimitSwitchPin)
    : aruwlib::control::Subsystem(drivers),
      leftClamped(false),
      rightClamped(false),
      LEFT_TOW_PIN(leftTowPin),
      RIGHT_TOW_PIN(rightTowPin),
      LEFT_TOW_LIMIT_SWITCH(leftTowLimitSwitchPin),
      RIGHT_TOW_LIMIT_SWITCH_PIN(rightTowLimitSwitchPin)
{
}

void TowSubsystem::setLeftClamped(bool isClamped)
{
    leftClamped = isClamped;
    drivers->digital.set(LEFT_TOW_PIN, leftClamped);
}

bool TowSubsystem::getLeftClamped() const { return leftClamped; }

void TowSubsystem::setRightClamped(bool isClamped)
{
    rightClamped = isClamped;
    drivers->digital.set(RIGHT_TOW_PIN, rightClamped);
}

bool TowSubsystem::getRightClamped() const { return rightClamped; }

bool TowSubsystem::getLeftLimitSwitchTriggered() const
{
    return drivers->digital.read(LEFT_TOW_LIMIT_SWITCH);
}

bool TowSubsystem::getRightLeftLimitSwitchTriggered() const
{
    return drivers->digital.read(RIGHT_TOW_LIMIT_SWITCH_PIN);
}
}  // namespace engineer
}  // namespace aruwsrc
