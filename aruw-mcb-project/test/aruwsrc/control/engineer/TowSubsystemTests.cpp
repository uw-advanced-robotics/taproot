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

#include "aruwsrc/control/engineer/TowSubsystem.hpp"

using aruwlib::Drivers;
using aruwlib::gpio::Digital;
using aruwsrc::engineer::TowSubsystem;

static constexpr Digital::OutputPin LEFT_TOW_PIN = Digital::OutputPin::E;
static constexpr Digital::OutputPin RIGHT_TOW_PIN = Digital::OutputPin::F;
static constexpr Digital::InputPin LEFT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::A;
static constexpr Digital::InputPin RIGHT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::B;

// get*clamped

TEST(TowSubsystem, getLeftClamped_default_returns_false)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);

    EXPECT_FALSE(ts.getLeftClamped());
}

TEST(TowSubsystem, getRightClamped_default_returns_false)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);

    EXPECT_FALSE(ts.getRightClamped());
}

// get*LimitSwitchTriggered

TEST(TowSubsystem, getLeftLimitSwitchTriggered_returns_false_when_digital_pin_low)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    EXPECT_CALL(drivers.digital, read(LEFT_TOW_LIMIT_SWITCH_PIN)).WillOnce([](Digital::InputPin) {
        return false;
    });

    EXPECT_FALSE(ts.getLeftLimitSwitchTriggered());
}

TEST(TowSubsystem, getLeftLimitSwitchTriggered_returns_true_when_digital_pin_high)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    EXPECT_CALL(drivers.digital, read(LEFT_TOW_LIMIT_SWITCH_PIN)).WillOnce([](Digital::InputPin) {
        return true;
    });

    EXPECT_TRUE(ts.getLeftLimitSwitchTriggered());
}

TEST(TowSubsystem, getRightLimitSwitchTriggered_returns_false_when_digital_pin_low)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    EXPECT_CALL(drivers.digital, read(RIGHT_TOW_LIMIT_SWITCH_PIN)).WillOnce([](Digital::InputPin) {
        return false;
    });

    EXPECT_FALSE(ts.getRightLeftLimitSwitchTriggered());
}

TEST(TowSubsystem, getRightLimitSwitchTriggered_returns_true_when_digital_pin_high)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    EXPECT_CALL(drivers.digital, read(RIGHT_TOW_LIMIT_SWITCH_PIN)).WillOnce([](Digital::InputPin) {
        return true;
    });

    EXPECT_TRUE(ts.getRightLeftLimitSwitchTriggered());
}

// set*Clamped and get*Clamped

TEST(TowSubsystem, setLeftClamped_false_results_in_getLeftClamped_returning_false)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    EXPECT_CALL(drivers.digital, set);

    ts.setLeftClamped(false);
    EXPECT_FALSE(ts.getLeftClamped());
}

TEST(TowSubsystem, setLeftClamped_true_results_in_getLeftClamped_returning_true)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    EXPECT_CALL(drivers.digital, set);

    ts.setLeftClamped(true);
    EXPECT_TRUE(ts.getLeftClamped());
}

TEST(TowSubsystem, setRightClamped_false_results_in_getRightClamped_returning_false)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    EXPECT_CALL(drivers.digital, set);

    ts.setRightClamped(false);
    EXPECT_FALSE(ts.getRightClamped());
}

TEST(TowSubsystem, setRightClamped_true_results_in_getRightClamped_returning_true)
{
    Drivers drivers;
    TowSubsystem ts(
        &drivers,
        LEFT_TOW_PIN,
        RIGHT_TOW_PIN,
        LEFT_TOW_LIMIT_SWITCH_PIN,
        RIGHT_TOW_LIMIT_SWITCH_PIN);
    EXPECT_CALL(drivers.digital, set);

    ts.setRightClamped(true);
    EXPECT_TRUE(ts.getRightClamped());
}
