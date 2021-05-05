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

#ifndef GRABBER_SUBSYSTEM_MOCK_HPP_
#define GRABBER_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/engineer/grabber_subsystem.hpp"

namespace aruwsrc::mock
{
class GrabberSubsystemMock : public engineer::GrabberSubsystem
{
    GrabberSubsystemMock(aruwlib::Drivers *drivers, aruwlib::gpio::Digital::OutputPin pin)
        : engineer::GrabberSubsystem(drivers, pin)
    {
    }

    MOCK_METHOD(void, setSqueezed, (bool), (override));
    MOCK_METHOD(bool, isSqueezed, (), (const override));
    MOCK_METHOD(void, runHardwareTests, (), (override));
    MOCK_METHOD(const char *, getName, (), (override));
};
}  // namespace aruwsrc::mock

#endif  // GRABBER_SUBSYSTEM_MOCK_HPP_
