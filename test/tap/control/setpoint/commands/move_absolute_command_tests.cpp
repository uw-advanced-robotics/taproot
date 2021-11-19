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
#include "tap/control/setpoint/commands/move_absolute_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/setpoint_subsystem_mock.hpp"

using tap::arch::clock::setTime;
using namespace tap::control::setpoint;
using tap::Drivers;
using namespace tap::mock;
using namespace testing;

#define CREATE_COMMON_TEST_OBJECTS() \
    Drivers drivers;                 \
    NiceMock<SetpointSubsystemMock> subsystem(&drivers);

// constructor tests ------------------------------------

/**
 * The MoveAbsoluteCommand requires sole access to the subsystem it
 * controls.
 */
TEST(MoveAbsoluteCommand, command_registers_subsystem_requirements)
{
    Drivers drivers;
    NiceMock<SetpointSubsystemMock> subsystem(&drivers);

    EXPECT_CALL(subsystem, getGlobalIdentifier).Times(AtLeast(1)).WillRepeatedly(Return(3));

    MoveAbsoluteCommand command(&subsystem, 0.0f, 0.0f, 0.0f, false, true);

    EXPECT_EQ(command.getRequirementsBitwise(), (1U << 3));
}

// isReady() tests ------------------------------------

TEST(MoveAbsoluteCommand, command_ready_when_subsystem_unjammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 0.0f, 0.0f, 0.0f, false, true);

    EXPECT_CALL(subsystem, isJammed).Times(AtLeast(1)).WillRepeatedly(Return(false));

    EXPECT_TRUE(command.isReady());
}

TEST(MoveAbsoluteCommand, command_is_not_ready_when_subsystem_jammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 0.0f, 0.0f, 0.0f, false, true);

    EXPECT_CALL(subsystem, isJammed).Times(AtLeast(1)).WillRepeatedly(Return(true));

    EXPECT_FALSE(command.isReady());
}

TEST(MoveAbsoluteCommand, command_is_not_ready_when_subsystem_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 0.0f, 0.0f, 0.0f, false, true);

    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(false));

    EXPECT_FALSE(command.isReady());
}

// initialize() tests ------------------------------------

// no explicit expectations set on initialize()

// execute() tests ------------------------------------

/**
 * Regardless of physical state of subsystem, move absolute command should set
 * setpoint to target after appropriate amount of time (distance / time).
 */
TEST(MoveAbsoluteCommand, command_sets_setpoint_to_target_after_appropriate_time)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 7.5f, 6.5f, 0.001f, false, true);

    EXPECT_CALL(subsystem, setSetpoint(FloatEq(7.5f)));
    // For setting up ramp or whatever command should ask where it's starting from at
    // least once (probably in initialize but we don't strictly require that)
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));

    setTime(0);
    command.initialize();
    setTime(1000);
    command.execute();
}

TEST(MoveAbsoluteCommand, command_does_not_reach_setpoint_given_too_little_time)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 7.5f, 6.5f, 0.001f, false, true);

    EXPECT_CALL(subsystem, setSetpoint(FloatEq(1.0f + 6.5f * 0.999f)));
    EXPECT_CALL(subsystem, setSetpoint(FloatEq(7.5f))).Times(0);
    // For setting up ramp or whatever command should ask where it's starting from at
    // least once (probably in initialize but we don't strictly require that)
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));

    setTime(0);
    command.initialize();
    setTime(999);
    command.execute();
}

/**
 * When the subsystem is offline we don't want the command to continue moving
 * the setpoint forward (as this would cause a large leap in the setpoint from
 * the subsystems POV, potentially requiring more jerky movement then desired).
 */
TEST(MoveAbsoluteCommand, command_sleeps_while_subsystem_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 7.5f, 6.5f, 0.001f, false, true);

    // setSetpoint() should only be set on the second execute, as during the first one
    // subsystem is offline.
    EXPECT_CALL(subsystem, setSetpoint).Times(1);
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));

    // subsystem online status should be checked every execution to determine whether or
    // not to execute regular logic.
    EXPECT_CALL(subsystem, isOnline).WillOnce(Return(false)).WillRepeatedly(Return(true));
    setTime(0);
    command.initialize();
    setTime(1000);
    command.execute();
    setTime(2000);
}

// isFinished() tests ------------------------------------

TEST(
    MoveAbsoluteCommand,
    command_finishes_once_current_value_within_tolerable_distance_to_target_and_ramp_finished)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 1.0f, 1.0f, 0.15f, false, true);

    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(0.9f));

    setTime(0);
    command.initialize();
    setTime(1);
    command.execute();
    setTime(99);
    command.execute();
    // At this point ramp hasn't been given enough time to reach target, so command should not be
    // finished
    EXPECT_FALSE(command.isFinished());
    // 100 ms * 1.0 units/second = displacement of 0.1 units which is all ramp should need from 0.9
    // to 1.0, add 1 ms to account for
    setTime(101);
    command.execute();
    EXPECT_TRUE(command.isFinished());
}

TEST(MoveAbsoluteCommand, command_finishes_if_subsystem_jammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 7.5f, 6.5f, 0.001f, false, true);

    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    ON_CALL(subsystem, isJammed).WillByDefault(Return(true));
    setTime(0);
    command.initialize();
    setTime(1);
    command.execute();
    EXPECT_TRUE(command.isFinished());
}

TEST(MoveAbsoluteCommand, command_finishes_if_subsystem_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 7.5f, 6.5f, 0.001f, false, true);

    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(false));
    setTime(0);
    command.isReady();
    command.initialize();
    setTime(1);
    command.execute();
    ASSERT_TRUE(command.isFinished());
}

TEST(MoveAbsoluteCommand, command_not_finished_when_system_unjammed_and_setpoint_not_reached)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 7.0f, 6.5f, 0.001f, false, true);

    ON_CALL(subsystem, isJammed).WillByDefault(Return(false));
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));

    setTime(0);
    command.initialize();
    setTime(100);
    command.execute();
    EXPECT_FALSE(command.isFinished());
}

// end() tests ------------------------------------

TEST(MoveAbsoluteCommand, command_sets_setpoint_to_target_on_end_when_option_set)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 7.5f, 6.5f, 0.001f, false, true);

    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    // To not have failure due to unexpected calls to setSetpoint, we have a catch-all expectation.
    // see: tip at bottom of
    // http://google.github.io/googletest/gmock_for_dummies.html#MultiExpectations
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, setSetpoint(FloatEq(7.5f))).Times(AtLeast(1));

    setTime(0);
    command.initialize();
    setTime(1);
    command.execute();
    command.end(false);
}

TEST(MoveAbsoluteCommand, command_sets_setpoint_to_current_value_on_end_when_option_set)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 2.0f, 6.5f, 0.001f, false, false);

    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, setSetpoint(FloatEq(1.0f))).Times(AtLeast(1));

    setTime(0);
    command.initialize();
    setTime(200);
    command.execute();
    command.end(false);
}

TEST(MoveAbsoluteCommand, command_clears_jam_on_end_if_option_set)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 1.0f, 6.5f, 0.001f, true, true);

    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, clearJam);

    setTime(0);
    command.initialize();
    setTime(1);
    command.execute();
    command.end(false);
}
