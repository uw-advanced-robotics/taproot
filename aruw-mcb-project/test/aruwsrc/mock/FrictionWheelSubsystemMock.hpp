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

#ifndef FRICTION_WHEEL_SUBSYSTEM_MOCK_HPP_
#define FRICTION_WHEEL_SUBSYSTEM_MOCK_HPP_

#include <aruwlib/Drivers.hpp>
#include <gmock/gmock.h>

#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class FrictionWheelSubsystemMock : public aruwsrc::launcher::FrictionWheelSubsystem
{
public:
    FrictionWheelSubsystemMock(aruwlib::Drivers *drivers);
    virtual ~FrictionWheelSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, setDesiredRpm, (float val), (override));
};  // class FrictionWheelSubsystemMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // FRICTION_WHEEL_SUBSYSTEM_MOCK_HPP_