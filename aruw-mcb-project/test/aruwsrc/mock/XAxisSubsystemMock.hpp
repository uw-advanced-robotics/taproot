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

#ifndef X_AXIS_SUBSYSTEM_MOCK_HPP_
#define X_AXIS_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/engineer/xaxis_subsystem.hpp"

namespace aruwsrc::mock
{
class XAxisSubsystemMock : public engineer::XAxisSubsystem
{
    XAxisSubsystemMock(aruwlib::Drivers *drivers, aruwlib::gpio::Digital::OutputPin pin);
    virtual ~XAxisSubsystemMock();

    MOCK_METHOD(void, setExtended, (bool), (override));
    MOCK_METHOD(bool, isExtended, (), (const override));
    MOCK_METHOD(void, runHardwareTests, (), (override));
    MOCK_METHOD(const char *, getName, (), (override));
};
}  // namespace aruwsrc::mock

#endif  // X_AXIS_SUBSYSTEM_MOCK_HPP_
