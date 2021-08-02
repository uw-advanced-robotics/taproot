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
#include "tap/control/setpoint/commands/move_absolute_command.hpp"
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

/**
 * The MoveAbsoluteCommand requires sole access to the subsystem it
 * controls.
 */
TEST(MoveAbsoluteCommand, command_registers_subsystem_requirements)
{
    Drivers drivers;
    NiceMock<SetpointSubsystemMock> subsystem(&drivers);

    EXPECT_CALL(subsystem, getGlobalIdentifier).Times(AtLeast(1)).WillRepeatedly(Return(3));

    MoveAbsoluteCommand command(&subsystem, 0.0f, 0.0f, 0.0f, false);

    EXPECT_EQ(command.getRequirementsBitwise(), (1U << 3));
}

// isReady() tests ------------------------------------

TEST(MoveAbsoluteCommand, command_is_only_ready_when_subsystem_online_and_unjammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 0.0f, 0.0f, 0.0f, false);
    
    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(true));
    EXPECT_CALL(subsystem, isJammed).Times(AtLeast(1)).WillRepeatedly(Return(false));

    EXPECT_TRUE(command.isReady());
}

TEST(MoveAbsoluteCommand, command_is_not_ready_when_subsystem_offline)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 0.0f, 0.0f, 0.0f, false);
    
    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(false));

    EXPECT_FALSE(command.isReady());
}

TEST(MoveAbsoluteCommand, command_is_not_ready_when_subsystem_jammed)
{
    CREATE_COMMON_TEST_OBJECTS();
    MoveAbsoluteCommand command(&subsystem, 0.0f, 0.0f, 0.0f, false);
    
    EXPECT_CALL(subsystem, isJammed).Times(AtLeast(1)).WillRepeatedly(Return(true));
    
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
    MoveAbsoluteCommand command(&subsystem, 7.5f, 6.5f, 0.0f, false);

    EXPECT_CALL(subsystem, setSetpoint(FloatEq(7.5f)));
    // For setting up ramp or whatever command should ask where it's starting from at
    // least once (probably in initialize but we don't strictly require that)
    EXPECT_CALL(subsystem, getCurrentValue).Times(AtLeast(1)).WillRepeatedly(Return(1.0f));

    setTime(0);
    command.initialize();
    setTime(1000);
    command.execute();
}