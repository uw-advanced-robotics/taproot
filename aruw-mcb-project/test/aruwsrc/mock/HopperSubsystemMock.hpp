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

#ifndef HOPPER_SUBSYSTEM_MOCK_HPP_
#define HOPPER_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/hopper-cover/hopper_subsystem.hpp"

namespace aruwsrc::mock
{
class HopperSubsystemMock : public control::HopperSubsystem
{
    HopperSubsystemMock(
        aruwlib::Drivers *drivers,
        aruwlib::gpio::Pwm::Pin pwmPin,
        float open,
        float close,
        float pwmRampSpeed)
        : control::HopperSubsystem(drivers, pwmPin, open, close, pwmRampSpeed)
    {
    }

    MOCK_METHOD(void, setOpen, (), (override));
    MOCK_METHOD(void, setClose, (), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(void, runHardwareTests, (), (override));
    MOCK_METHOD(const char *, getName, (), (override));
};
}  // namespace aruwsrc::mock

#endif  // HOPPER_SUBSYSTEM_MOCK_HPP_
