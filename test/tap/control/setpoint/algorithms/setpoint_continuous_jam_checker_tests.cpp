/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/control/setpoint/algorithms/setpoint_continuous_jam_checker.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/setpoint_subsystem_mock.hpp"

using namespace tap::arch::clock;
using namespace tap::control::setpoint;
using tap::Drivers;
using namespace tap::mock;
using namespace testing;

TEST(SetpointContinuousJamChecker, detects_non_jams_correctly)
{
    ClockStub clock;
    Drivers drivers;
    NiceMock<SetpointSubsystemMock> subsystem(&drivers);
    SetpointContinuousJamChecker jamChecker(&subsystem, 0.6f, 200);

    EXPECT_CALL(subsystem, getCurrentValue).WillRepeatedly(Return(3.0f));
    EXPECT_CALL(subsystem, getSetpoint).WillRepeatedly(Return(2.5f));

    clock.time = 0;
    jamChecker.restart();
    clock.time = 300;
    EXPECT_FALSE(jamChecker.check());
}

TEST(SetpointContinuousJamChecker, detects_jams_correctly)
{
    ClockStub clock;
    Drivers drivers;
    NiceMock<SetpointSubsystemMock> subsystem(&drivers);
    SetpointContinuousJamChecker jamChecker(&subsystem, 0.4f, 200);

    EXPECT_CALL(subsystem, getCurrentValue).WillRepeatedly(Return(3.0f));
    EXPECT_CALL(subsystem, getSetpoint).WillRepeatedly(Return(2.5f));

    clock.time = 0;
    jamChecker.restart();
    clock.time = 200;
    EXPECT_TRUE(jamChecker.check());
}

// Jam timer shouldn't be started just by constructing it. That would be annoying.
TEST(SetpointContinuousJamChecker, jam_timer_not_started_at_construction)
{
    ClockStub clock;
    Drivers drivers;
    NiceMock<SetpointSubsystemMock> subsystem(&drivers);
    clock.time = 0;
    SetpointContinuousJamChecker jamChecker(&subsystem, 0.4f, 200);

    EXPECT_CALL(subsystem, getCurrentValue).WillRepeatedly(Return(3.0f));
    EXPECT_CALL(subsystem, getSetpoint).WillRepeatedly(Return(2.5f));

    clock.time = 300;
    EXPECT_FALSE(jamChecker.check());
}
