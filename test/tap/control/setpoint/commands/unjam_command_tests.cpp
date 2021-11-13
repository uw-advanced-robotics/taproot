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
#include "tap/control/setpoint/commands/unjam_command.hpp"
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

TEST(UnjamCommand, command_registers_subsystem_requirements)
{
    CREATE_COMMON_TEST_OBJECTS();
    EXPECT_CALL(subsystem, getGlobalIdentifier).Times(AtLeast(1)).WillRepeatedly(Return(3));
    UnjamCommand command(&subsystem, 3.0f, 2.5f, 400, 2);

    EXPECT_EQ(command.getRequirementsBitwise(), (1U << 3));
}

// isReady() tests ------------------------------------

// No expectations on isReady. UnjamCommand should be able
// to run whenever

// initialize() tests ------------------------------------

/**
 * During initialization the command should request the subsystem's
 * current value to determine where to offset from.
 */
TEST(UnjamCommand, command_gets_startpoint)
{
    CREATE_COMMON_TEST_OBJECTS();
    UnjamCommand command(&subsystem, 3.0f, 2.5f, 400, 2);

    EXPECT_CALL(subsystem, getCurrentValue()).Times(AtLeast(1));

    command.initialize();
}

// execution tests ------------------------------------

TEST(UnjamCommand, command_moves_subsystem_back_and_forth_appropriate_number_of_times)
{
    CREATE_COMMON_TEST_OBJECTS();
    constexpr int NUM_CYCLES = 5;
    UnjamCommand command(&subsystem, 3.0f, 2.5f, 400, NUM_CYCLES);

    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(-10.0f));
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, setSetpoint(Ge(-7.5f))).Times(NUM_CYCLES);
    EXPECT_CALL(subsystem, setSetpoint(Le(-12.5f))).Times(NUM_CYCLES + 1);

    setTime(0);
    ASSERT_TRUE(command.isReady());
    command.initialize();
    command.execute();
    int i = 200;
    while (i <= 10000 && !command.isFinished())
    {
        setTime(i);
        command.execute();
        i += 200;
    }
}

TEST(UnjamCommand, command_does_not_execute_while_subsystem_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    UnjamCommand command(&subsystem, 3.0f, 2.5f, 400, 2);

    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(false));
    // Allow setpoint to be set at most once during initialization step
    EXPECT_CALL(subsystem, setSetpoint).Times(AtMost(1));

    setTime(0);
    command.initialize();

    for (int i = 200; i <= 3000; i += 200)
    {
        setTime(i);
        command.execute();
    }
}

// isFinished() tests ------------------------------------

// Test function for following test
static float getCurrentValueSimulator()
{
    uint32_t time = getTimeMilliseconds();
    // Return initial position
    if (time <= 100)
    {
        return 2.5f;
    }
    else if (time <= 400)
    {
        return 0.0f;
    }
    else
    {
        return 5.0f;
    }
}

/**
 * Subsystem will be determined to be unjammed when unjam command
 * was able to successfully reach backwards and forwards `unjamDisplacement`
 */
TEST(UnjamCommand, command_unjams_subsystem_when_unjam_displacement_reached_in_both_directions)
{
    CREATE_COMMON_TEST_OBJECTS();
    UnjamCommand command(&subsystem, 3.0f, 2.4f, 1000, 2);

    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(true));
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, getCurrentValue).WillRepeatedly(getCurrentValueSimulator);
    EXPECT_CALL(subsystem, clearJam).Times(AtLeast(1));

    setTime(0);
    command.initialize();

    for (int i = 200; i <= 1200; i += 200)
    {
        setTime(i);
        command.execute();
    }

    // command.isFinished() called here to model actual command lifecycle
    // but should have no effect on command's state as it is a const function
    command.isFinished();

    command.end(true);

}

// end() tests ------------------------------------

/**
 * An old implementation of end() had the command clear jam there.
 * This is an issue as end() can be called by having the command
 * interrupted. This is to check that that isn't happening
 */
TEST(UnjamCommand, command_does_NOT_clear_jam_on_end)
{
    CREATE_COMMON_TEST_OBJECTS();
    UnjamCommand command(&subsystem, 3.0f, 2.5f, 400, 2);

    EXPECT_CALL(subsystem, clearJam).Times(0);

    setTime(0);
    command.initialize();

    setTime(1);
    command.execute();

    // Oh no! Command has been interrupted and ended, clearJam shouldn't be
    // called.
    setTime(2);
    command.end(true);
}

TEST(UnjamCommand, command_resets_setpoint_on_end)
{
    CREATE_COMMON_TEST_OBJECTS();
    UnjamCommand command(&subsystem, 3.0f, 2.5f, 400, 2);

    EXPECT_CALL(subsystem, getSetpoint).WillRepeatedly(Return(-10.0f));
    EXPECT_CALL(subsystem, setSetpoint).Times(AnyNumber());
    EXPECT_CALL(subsystem, setSetpoint(FloatEq(-10.0f)));

    setTime(0);
    command.initialize();
    for (int i = 50; i <= 1000; i += 50)
    {
        setTime(i);
        command.execute();
    }

    command.end(!command.isFinished());
}
