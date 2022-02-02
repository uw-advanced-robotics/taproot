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
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/setpoint_subsystem_mock.hpp"

using namespace tap::arch::clock;
using namespace tap::control::setpoint;
using tap::Drivers;
using namespace tap::mock;
using namespace testing;

#define CREATE_COMMON_TEST_OBJECTS() \
    Drivers drivers;                 \
    NiceMock<SetpointSubsystemMock> subsystem(&drivers);

// constructor tests ------------------------------------

TEST(MoveUnjamComprisedCommand, command_registers_subsystem_requirements)
{
    CREATE_COMMON_TEST_OBJECTS();
    EXPECT_CALL(subsystem, getGlobalIdentifier).Times(AtLeast(1)).WillRepeatedly(Return(3));
    MoveUnjamComprisedCommand
        command(&drivers, &subsystem, 5.0f, 1000, 30, true, 0.2f, 3.0f, 2.5f, 500, 3);

    EXPECT_EQ(command.getRequirementsBitwise(), (1U << 3));
}

// isReady() tests ------------------------------------

// Command should basically always be ready
TEST(MoveUnjamComprisedCommand, command_ready_even_when_subsystem_jammed_and_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveUnjamComprisedCommand
        command(&drivers, &subsystem, 5.0f, 1000, 30, true, 0.2f, 3.0f, 2.5f, 500, 3);

    ON_CALL(subsystem, isJammed).WillByDefault(Return(true));
    ON_CALL(subsystem, isOnline).WillByDefault(Return(true));

    EXPECT_TRUE(command.isReady());
}

// initialize() tests ------------------------------------

// No expectations on initialize

// execute() tests ------------------------------------

// Copied from move command tests
TEST(MoveUnjamComprisedCommand, command_displaces_setpoint_by_target_amount_after_sufficient_time)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveUnjamComprisedCommand
        command(&drivers, &subsystem, 7.5f, 1000, 15, true, 0.2f, 3.0f, 2.5f, 500, 3);

    // Move command is relative to previous setpoint and must check current value
    // to see where it's starting from.
    float currentValue = 1.0f;
    EXPECT_CALL(subsystem, getSetpoint).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));
    EXPECT_CALL(subsystem, getCurrentValue)
        .Times(AtLeast(1))
        .WillRepeatedly(ReturnPointee(&currentValue));
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, setSetpoint(FloatNear(8.5f, 0.1f))).Times(AtLeast(1));

    setTime(0);
    command.initialize();
    for (int i = 200; i <= 1000; i += 200)
    {
        setTime(i);
        command.execute();
    }

    currentValue = 8.5f;
    command.execute();

    // Insufficient time given for pause after rotate time
    EXPECT_FALSE(command.isFinished());

    // Account for pauseAfterRotateTime
    setTime(1015);
    currentValue = 8.5f;
    command.execute();

    // Once setpoint is reached and pause after rotate time is reached command should finish
    EXPECT_TRUE(command.isFinished());
}

// Test function for following test
static float getCurrentValueSimulator()
{
    uint32_t time = getTimeMilliseconds();
    // Return initial position
    if (time <= 100)
    {
        return 3.0f;
    }
    else if (time <= 400)
    {
        return 0.0f;
    }
    else if (time <= 800)
    {
        return 6.0f;
    }
    else
    {
        return 3.0f;
    }
}

/**
 * This test is fundamentally flawed as
 */
TEST(MoveUnjamComprisedCommand, command_attempts_to_unjam_when_jammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveUnjamComprisedCommand
        command(&drivers, &subsystem, 7.5f, 1000, 15, true, 0.2f, 3.0f, 2.5f, 500, 3);

    bool jamStatus = true;
    EXPECT_CALL(subsystem, isJammed).WillRepeatedly(ReturnPointee(&jamStatus));
    EXPECT_CALL(subsystem, getSetpoint).WillRepeatedly(Return(3.0f));
    EXPECT_CALL(subsystem, getCurrentValue).WillRepeatedly(getCurrentValueSimulator);
    EXPECT_CALL(subsystem, getJamSetpointTolerance).WillRepeatedly(Return(0.01f));
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, setSetpoint(Lt(0.1f))).Times(AtLeast(1));
    EXPECT_CALL(subsystem, setSetpoint(Gt(5.9f))).Times(AtLeast(1));
    EXPECT_CALL(subsystem, clearJam).Times(1).WillRepeatedly(Assign(&jamStatus, false));

    setTime(0);
    command.initialize();
    // Note: time between clearing last thing needed to clear (happens for us at 400)
    // and calling end must be less than max unjam rotate time, otherwise next call
    // of execute() causes unjam to think it's failed to return to origin.
    for (int i = 50; i <= 1200; i += 50)
    {
        setTime(i);
        command.execute();
        if (command.isFinished())
        {
            break;
        }
    }

    command.end(!command.isFinished());
}

TEST(MoveUnjamComprisedCommand, command_does_not_execute_while_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveUnjamComprisedCommand
        command(&drivers, &subsystem, 7.5f, 1000, 15, true, 0.2f, 3.0f, 2.5f, 500, 3);

    EXPECT_CALL(subsystem, isOnline).WillRepeatedly(Return(false));
    EXPECT_CALL(subsystem, setSetpoint).Times(0);

    setTime(0);
    command.initialize();
    for (int i = 200; i <= 1000; i += 200)
    {
        setTime(i);
        command.execute();
    }
}

// No end() or isFinished() tests. Logic for isFinished tested above
