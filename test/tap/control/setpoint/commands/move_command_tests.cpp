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
#include "tap/drivers.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/mock/setpoint_subsystem_mock.hpp"

using tap::arch::clock::setTime;
using namespace tap::control::setpoint;
using tap::Drivers;
using namespace tap::mock;
using namespace testing;

#define CREATE_COMMON_TEST_OBJECTS()                     \
    Drivers drivers;                                     \
    NiceMock<SetpointSubsystemMock> subsystem(&drivers);

// constructor tests ------------------------------------

TEST(MoveCommand, command_registers_subsystem_requirements)
{
    CREATE_COMMON_TEST_OBJECTS();
    EXPECT_CALL(subsystem, getGlobalIdentifier)
        .Times(AtLeast(1))
        .WillRepeatedly(Return(3));
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 0.001f);

    EXPECT_EQ(command.getRequirementsBitwise(), (1U << 3));
}

// isReady() tests ------------------------------------

TEST(MoveCommand, command_not_ready_when_subsystem_jammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 0.001f);

    EXPECT_CALL(subsystem, isJammed).Times(AtLeast(1)).WillRepeatedly(Return(true));

    EXPECT_FALSE(command.isReady());
}

TEST(MoveCommand, command_ready_when_subsystem_unjammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 0.001f);

    EXPECT_CALL(subsystem, isJammed).Times(AtLeast(1)).WillRepeatedly(Return(false));

    EXPECT_TRUE(command.isReady());
}

TEST(MoveCommand, command_not_ready_when_subsystem_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 0.001f);

    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(false));

    ASSERT_FALSE(command.isReady());
}

// initialize() tests ------------------------------------

// No explicit expectations set on initialize()

// execute() tests ------------------------------------

TEST(MoveCommand, command_displaces_setpoint_by_target_amount_after_sufficient_time)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 0.001f);

    // Move command is relative to previous setpoint and must check current value
    // to see where it's starting from.
    EXPECT_CALL(subsystem, getSetpoint).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, setSetpoint(FloatNear(8.5f, 0.1f)));

    setTime(0);
    command.initialize();
    for (int i = 200; i <= 1000; i += 200)
    {
        setTime(i);
        command.execute();
    }
}

TEST(MoveCommand, command_does_not_reach_setpoint_given_insufficient_time)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 0.001f);

    EXPECT_CALL(subsystem, getSetpoint).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, setSetpoint(Ge(8.49999f))).Times(0);

    setTime(0);
    command.initialize();
    for (int i = 200; i <= 800; i += 200)
    {
        setTime(i);
        command.execute();
    }
    setTime(995);
    command.execute();
}

TEST(MoveCommand, command_does_not_execute_while_subsystem_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 0.001f);

    EXPECT_CALL(subsystem, getSetpoint).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(false));
    EXPECT_CALL(subsystem, setSetpoint).Times(0);

    setTime(0);
    command.initialize();
    for (int i = 200; i <= 1000; i += 200)
    {
        setTime(i);
        command.execute();
    }
}

// isFinished() tests ------------------------------------

TEST(MoveCommand, command_is_finished_when_subsystem_jammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 0.001f);

    EXPECT_CALL(subsystem, isJammed)
        .Times(AtLeast(1))
        .WillRepeatedly(Return(true));

    setTime(0);
    command.initialize();
    setTime(200);
    command.execute();
    EXPECT_TRUE(command.isFinished());
}

TEST(MoveCommand, command_is_finished_when_subsystem_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 0.001f);

    setTime(0);
    command.isReady();
    command.initialize();
    setTime(200);
    command.execute();

    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(false));

    ASSERT_TRUE(command.isFinished());
}

TEST(MoveCommand, command_is_finished_when_subsystem_unjammed_and_displacement_within_tolerance)
{
    // Command being finished also requires sufficient time to have passed.
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 1.0f);

    EXPECT_CALL(subsystem, getSetpoint).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(8.5f));
    EXPECT_CALL(subsystem, isJammed)
        .Times(AtLeast(1))
        .WillRepeatedly(Return(false));

    setTime(0);
    command.initialize();
    // When command executes it should see that subsystem position is at target and start pause after
    // rotate timeout
    command.execute();
    // Provide sufficient time for pause after rotation
    setTime(15);
    EXPECT_TRUE(command.isFinished());
}

TEST(MoveCommand, command_pauses_after_move_time)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 1.0f);

    EXPECT_CALL(subsystem, getSetpoint).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(8.5f));
    EXPECT_CALL(subsystem, isJammed)
        .Times(AtLeast(1))
        .WillRepeatedly(Return(false));

    setTime(0);
    command.initialize();
    setTime(1000);
    command.execute();
    EXPECT_FALSE(command.isFinished());
    setTime(1015);
    EXPECT_TRUE(command.isFinished());
}

// end() tests ------------------------------------

TEST(MoveCommand, command_sets_setpoint_to_ideal_target_on_end_when_option_enabled)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 1.0f);

    EXPECT_CALL(subsystem, getSetpoint).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, setSetpoint(FloatEq(8.5f)));

    setTime(0);
    command.initialize();
    setTime(10);
    command.end(true);
}

TEST(MoveCommand, command_sets_setpoint_to_current_value_on_end_when_jammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, true, 1.0f);

    EXPECT_CALL(subsystem, isJammed).Times(AtLeast(1)).WillRepeatedly(Return(true));
    EXPECT_CALL(subsystem, getSetpoint).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(50.0f));
    EXPECT_CALL(subsystem, setSetpoint(FloatEq(50.0f)));

    setTime(0);
    command.initialize();
    setTime(10);
    command.end(false);
}

TEST(MoveCommand, command_sets_setpoint_to_current_value_on_end_based_on_option)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveCommand command(&subsystem, 7.5f, 1000, 15, false, 1.0f);

    EXPECT_CALL(subsystem, getSetpoint).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(50.0f));
    EXPECT_CALL(subsystem, setSetpoint(FloatEq(50.0f)));

    setTime(0);
    command.initialize();
    setTime(10);
    command.end(false);
}
