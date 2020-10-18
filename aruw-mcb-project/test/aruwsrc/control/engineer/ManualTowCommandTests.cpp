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

#include <aruwlib/Drivers.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "aruwsrc/control/engineer/ManualTowCommand.hpp"
#include "aruwsrc/mock/TowSubsystemMock.hpp"

using aruwlib::Drivers;
using aruwlib::gpio::Digital;
using aruwsrc::engineer::ManualTowCommand;
using aruwsrc::mock::TowSubsystemMock;

static constexpr Digital::OutputPin LEFT_TOW_PIN = Digital::OutputPin::E;
static constexpr Digital::OutputPin RIGHT_TOW_PIN = Digital::OutputPin::F;
static constexpr Digital::InputPin LEFT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::A;
static constexpr Digital::InputPin RIGHT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::B;

TEST(ManualTowCommand, isFinished_always_returns_false)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    ManualTowCommand tc(&ts);

    EXPECT_FALSE(tc.isFinished());
}

TEST(ManualTowCommand, initialize_sets_left_and_right_clamped_true)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    ManualTowCommand tc(&ts);
    EXPECT_CALL(ts, setLeftClamped(true));
    EXPECT_CALL(ts, setRightClamped(true));

    tc.initialize();
}

TEST(ManualTowCommand, end_sets_left_and_right_clamped_false)
{
    Drivers drivers;
    TowSubsystemMock ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    ManualTowCommand tc(&ts);
    EXPECT_CALL(ts, setLeftClamped(false)).Times(2);
    EXPECT_CALL(ts, setRightClamped(false)).Times(2);

    tc.end(false);
    tc.end(true);
}
