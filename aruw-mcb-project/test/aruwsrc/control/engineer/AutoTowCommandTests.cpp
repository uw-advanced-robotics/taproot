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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "aruwlib/Drivers.hpp"

#include "aruwsrc/control/engineer/AutoTowCommand.hpp"
#include "aruwsrc/mock/TowSubsystemMock.hpp"

using aruwlib::Drivers;
using aruwlib::gpio::Digital;
using aruwsrc::engineer::AutoTowCommand;
using aruwsrc::mock::TowSubsystemMock;

static constexpr Digital::OutputPin LEFT_TOW_PIN = Digital::OutputPin::E;
static constexpr Digital::OutputPin RIGHT_TOW_PIN = Digital::OutputPin::F;
static constexpr Digital::InputPin LEFT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::A;
static constexpr Digital::InputPin RIGHT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::B;

TEST(AutoTowCommand, execute_dont_trigger_auto_tow_when_neither_switches_enabled)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    AutoTowCommand tc(&ts);
    EXPECT_CALL(ts, setLeftClamped).Times(0);
    EXPECT_CALL(ts, setRightClamped).Times(0);
    EXPECT_CALL(ts, getLeftClamped).WillOnce([] { return false; });
    EXPECT_CALL(ts, getRightClamped).WillOnce([] { return false; });
    EXPECT_CALL(ts, getLeftLimitSwitchTriggered).WillOnce([] { return false; });
    EXPECT_CALL(ts, getRightLeftLimitSwitchTriggered).WillOnce([] { return false; });

    tc.execute();
}

TEST(AutoTowCommand, execute_trigger_left_clamp_when_left_switch_enabled)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    AutoTowCommand tc(&ts);
    EXPECT_CALL(ts, setLeftClamped(true)).Times(1);
    EXPECT_CALL(ts, setRightClamped).Times(0);
    EXPECT_CALL(ts, getLeftClamped).WillOnce([] { return false; });
    EXPECT_CALL(ts, getRightClamped).WillOnce([] { return false; });
    EXPECT_CALL(ts, getLeftLimitSwitchTriggered).WillOnce([] { return true; });
    EXPECT_CALL(ts, getRightLeftLimitSwitchTriggered).WillOnce([] { return false; });

    tc.execute();
}

TEST(AutoTowCommand, execute_trigger_right_clamp_when_right_switch_enabled)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    AutoTowCommand tc(&ts);
    EXPECT_CALL(ts, setLeftClamped).Times(0);
    EXPECT_CALL(ts, setRightClamped(true)).Times(1);
    EXPECT_CALL(ts, getLeftClamped).WillOnce([] { return false; });
    EXPECT_CALL(ts, getRightClamped).WillOnce([] { return false; });
    EXPECT_CALL(ts, getLeftLimitSwitchTriggered).WillOnce([] { return false; });
    EXPECT_CALL(ts, getRightLeftLimitSwitchTriggered).WillOnce([] { return true; });

    tc.execute();
}

TEST(AutoTowCommand, execute_trigger_both_clamps_when_both_switches_enabled)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    AutoTowCommand tc(&ts);
    EXPECT_CALL(ts, setLeftClamped(true)).Times(1);
    EXPECT_CALL(ts, setRightClamped(true)).Times(1);
    EXPECT_CALL(ts, getLeftClamped).WillOnce([] { return false; });
    EXPECT_CALL(ts, getRightClamped).WillOnce([] { return false; });
    EXPECT_CALL(ts, getLeftLimitSwitchTriggered).WillOnce([] { return true; });
    EXPECT_CALL(ts, getRightLeftLimitSwitchTriggered).WillOnce([] { return true; });

    tc.execute();
}

TEST(AutoTowCommand, execute_dont_trigger_both_clamps_when_both_clamps_already_enabled)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    AutoTowCommand tc(&ts);
    EXPECT_CALL(ts, setLeftClamped).Times(0);
    EXPECT_CALL(ts, setRightClamped).Times(0);
    EXPECT_CALL(ts, getLeftClamped).WillOnce([] { return true; });
    EXPECT_CALL(ts, getRightClamped).WillOnce([] { return true; });

    tc.execute();
}

TEST(AutoTowCommand, end_always_sets_both_clamps_open)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    AutoTowCommand tc(&ts);
    EXPECT_CALL(ts, setLeftClamped(false)).Times(2);
    EXPECT_CALL(ts, setRightClamped(false)).Times(2);

    tc.end(false);
    tc.end(true);
}

TEST(AutoTowCommand, isFinished_always_returns_false)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    AutoTowCommand tc(&ts);

    EXPECT_FALSE(tc.isFinished());
}
