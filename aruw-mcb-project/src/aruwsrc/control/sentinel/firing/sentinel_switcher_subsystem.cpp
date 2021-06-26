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

#include "sentinel_switcher_subsystem.hpp"

#include <aruwsrc/control/agitator/agitator_subsystem.hpp>

#include "sentinel_switcher_subsystem.hpp"

namespace aruwsrc::control::sentinel::firing
{
SentinelSwitcherSubsystem::SentinelSwitcherSubsystem(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Pwm::Pin switcherServoPin)
    : aruwlib::control::Subsystem(drivers),
      switcherMotor(drivers, switcherServoPin, LOWER_PWM, UPPER_PWM, 0.1f)
{
    useLowerBarrel(this->useLower);
}

void SentinelSwitcherSubsystem::refresh() { switcherMotor.updateSendPwmRamp(); }

void SentinelSwitcherSubsystem::useLowerBarrel(bool useLower)
{
    switcherMotor.setTargetPwm(useLower ? LOWER_PWM : UPPER_PWM);
    this->useLower = useLower;
}

bool SentinelSwitcherSubsystem::isLowerUsed() const { return useLower; }
}  // namespace aruwsrc::control::sentinel::firing
