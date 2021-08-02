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

#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/mock/setpoint_subsystem_mock.hpp"

using namespace tap::control::setpoint;
using tap::Drivers;
using namespace tap::mock;
using namespace testing;

#define CREATE_COMMON_TEST_OBJECTS()                     \
    Drivers drivers;                                     \
    NiceMock<SetpointSubsystemMock> subsystem(&drivers); \
    CalibrateCommand command(&subsystem);

// constructor tests ------------------------------------

/**
 *  Even though the calibrate command shouldn't affect physical state
 * of subsystem, it should still have sole access to subsystem while
 * run as it messes with the setpoint and current value of the subsystem.
 * (Which would mess up other commands)
 */
TEST(CalibrateCommand, command_registers_subsystem_as_requirement)
{
    Drivers drivers;
    NiceMock<SetpointSubsystemMock> subsystem(&drivers);

    EXPECT_CALL(subsystem, getGlobalIdentifier).Times(AtLeast(1)).WillRepeatedly(Return(3));

    CalibrateCommand command(&subsystem);

    EXPECT_EQ(command.getRequirementsBitwise(), (1U << 3));
}

// isReady() tests ------------------------------------

/**
 * Calibrate command should only run when subsystem is online as otherwise
 * current value may be out of date.
 */
TEST(CalibrateCommand, command_is_not_ready_if_subsystem_offline)
{
    CREATE_COMMON_TEST_OBJECTS();

    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(false));

    EXPECT_FALSE(command.isReady());
}

TEST(CalibrateCommand, command_is_ready_if_subsystem_online)
{
    CREATE_COMMON_TEST_OBJECTS();

    EXPECT_CALL(subsystem, isOnline).Times(AtLeast(1)).WillRepeatedly(Return(true));

    EXPECT_TRUE(command.isReady());
}

// initialize() tests ------------------------------------

// none, no expected behavior for initialize

// execute() tests ------------------------------------

TEST(CalibrateCommand, execute_calls_subsystem_calibrate_function)
{
    CREATE_COMMON_TEST_OBJECTS();

    EXPECT_CALL(subsystem, calibrateHere);

    command.execute();
}

// isFinished() tests ------------------------------------

TEST(CalibrateCommand, command_not_finished_when_calibrations_unsuccessful)
{
    CREATE_COMMON_TEST_OBJECTS();

    // Simulate subsystem that is calibrated but calibration attempt fails
    ON_CALL(subsystem, isCalibrated).WillByDefault(Return(true));
    EXPECT_CALL(subsystem, calibrateHere).Times(AtLeast(1)).WillRepeatedly(Return(false));

    command.initialize();
    command.execute();

    EXPECT_FALSE(command.isFinished());
}

TEST(CalibrateCommand, command_finished_when_subsystem_calibrated_and_calibration_successful)
{
    CREATE_COMMON_TEST_OBJECTS();

    EXPECT_CALL(subsystem, isCalibrated).Times(AtLeast(1)).WillRepeatedly(Return(true));
    EXPECT_CALL(subsystem, calibrateHere).Times(AtLeast(1)).WillRepeatedly(Return(true));

    command.initialize();
    command.execute();
    EXPECT_TRUE(command.isFinished());
}

// end() tests ------------------------------------

// No expected behavior on end

// Overall tests ------------------------------------

TEST(CalibrateCommand, command_calibrates_until_successful)
{
    CREATE_COMMON_TEST_OBJECTS();

    ON_CALL(subsystem, isCalibrated).WillByDefault(Return(true));
    EXPECT_CALL(subsystem, calibrateHere)
        .WillOnce(Return(false))
        .WillOnce(Return(false))
        .WillRepeatedly(Return(true));
    
    // Used to break out of potential infinite loops
    int loopCounter = 0;

    command.initialize();
    command.execute();
    while (loopCounter < 4 && !command.isFinished())
    {
        command.execute();
        loopCounter++;
    }

    EXPECT_EQ(loopCounter, 2);
}
