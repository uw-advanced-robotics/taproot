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

#ifndef SENTINEL_SWITCHER_SUBSYSTEM_HPP_
#define SENTINEL_SWITCHER_SUBSYSTEM_HPP_

#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/servo.hpp>

#include "util_macros.hpp"

namespace aruwsrc
{
namespace sentinel
{
/**
 * Controls a servo used to switch which barrel balls are fed into. Barrel ID1 is assumed
 * to be on the bottom and barrel ID2 is assumed to be the top.
 */
class SentinelSwitcherSubsystem : public aruwlib::control::Subsystem
{
public:
    SentinelSwitcherSubsystem(aruwlib::Drivers *drivers, aruwlib::gpio::Pwm::Pin switcherServoPin);

    const char *getName() override { return "sentinel switcher"; }

    void refresh() override;

    mockable void useLowerBarrel(bool useLower);

    mockable bool isLowerUsed() const;

private:
    static constexpr float UPPER_PWM = 0.13f;
    static constexpr float LOWER_PWM = 0.19f;

    aruwlib::motor::Servo switcherMotor;
    bool useLower = true;
};

}  // namespace sentinel

}  // namespace aruwsrc

#endif
