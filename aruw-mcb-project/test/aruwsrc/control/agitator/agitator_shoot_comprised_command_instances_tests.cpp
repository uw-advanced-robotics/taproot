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

/**
 * The only tests performed here are to validate heat limiting is performed
 * properly.
 */

#include <gtest/gtest.h>

#include "aruwsrc/control/agitator/agitator_shoot_comprised_command_instances.hpp"
#include "aruwsrc/mock/AgitatorSubsystemMock.hpp"

using namespace aruwlib;
using namespace aruwsrc::agitator;
using namespace aruwsrc::mock;
using namespace testing;
using namespace aruwlib::serial;

#define SETUP_TEST(ComprisedCommand, ...)                              \
    Drivers drivers;                                                   \
    AgitatorSubsystemMock agitator(&drivers);                          \
    ComprisedCommand shootCommand(&drivers, &agitator, ##__VA_ARGS__); \
    EXPECT_CALL(drivers.djiMotorTxHandler, removeFromMotorManager);    \
    EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler);

#define SETUP_EXPECTATIONS_SHARED(startingAngle, receivingRefSerial)             \
    EXPECT_CALL(drivers.refSerial, getRobotData).WillOnce(ReturnRef(robotData)); \
    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData).WillOnce(Return(receivingRefSerial));

#define SETUP_EXPECTATIONS_NORMAL(startingAngle, receivingRefSerial)                        \
    SETUP_EXPECTATIONS_SHARED(startingAngle, receivingRefSerial);                           \
    EXPECT_CALL(agitator, getAgitatorAngle).Times(2).WillRepeatedly(Return(startingAngle)); \
    EXPECT_CALL(agitator, getAgitatorDesiredAngle).WillOnce(Return(startingAngle));         \
    EXPECT_CALL(agitator, armAgitatorUnjamTimer);

#define SETUP_EXPECTATIONS_OVERHEAT(startingAngle)  \
    SETUP_EXPECTATIONS_SHARED(startingAngle, true); \
    EXPECT_CALL(agitator, getAgitatorAngle).WillOnce(Return(startingAngle));

#define RUN_TEST(expectedFinished, finalDesiredAngle, overHeatLimit)                    \
    shootCommand.initialize();                                                          \
    EXPECT_CALL(agitator, getAgitatorDesiredAngle).WillOnce(Return(finalDesiredAngle)); \
    EXPECT_EQ(expectedFinished, shootCommand.isFinished());

// Validation for heat limiting confirmed by whether or not the command reports it is finished
// after initialize is called

TEST(
    ShootFastComprisedCommand17MM,
    command_updates_agitator_setpoint_if_heat_limiting_enabled_and_below_tolerance_start_angle_0)
{
    // Start curr agitator angle at 0
    SETUP_TEST(ShootFastComprisedCommand17MM, true);
    RefSerial::RobotData robotData;
    robotData.turret.heat17ID1 = 10;
    robotData.turret.heatLimit17ID1 = 100;
    SETUP_EXPECTATIONS_NORMAL(0, true);
    RUN_TEST(false, 10, false);
}

TEST(
    ShootFastComprisedCommand17MM,
    command_updates_agitator_setpoint_if_heat_limiting_enabled_and_below_tolerance_start_angle_10)
{
    SETUP_TEST(ShootFastComprisedCommand17MM, true);
    RefSerial::RobotData robotData;
    robotData.turret.heat17ID1 = 10;
    robotData.turret.heatLimit17ID1 = 100;
    SETUP_EXPECTATIONS_NORMAL(10, true);
    RUN_TEST(false, 20, false);
}

TEST(
    ShootFastComprisedCommand17MM,
    command_doesnt_update_agitator_setpoint_if_heat_limiting_enabled_and_within_tolerance)
{
    SETUP_TEST(ShootFastComprisedCommand17MM, true);
    RefSerial::RobotData robotData;
    robotData.turret.heat17ID1 = 10;
    robotData.turret.heatLimit17ID1 = 15;
    SETUP_EXPECTATIONS_OVERHEAT(0);
    RUN_TEST(true, 10, true);
}

TEST(
    ShootFastComprisedCommand17MM,
    command_doesnt_update_agitator_setpoint_if_heat_limiting_enabled_and_above_tolerance)
{
    SETUP_TEST(ShootFastComprisedCommand17MM, true);
    RefSerial::RobotData robotData;
    robotData.turret.heat17ID1 = 100;
    robotData.turret.heatLimit17ID1 = 95;
    SETUP_EXPECTATIONS_OVERHEAT(0);
    RUN_TEST(true, 10, true);
}

// Same cases as above but with heat limiting turned off

TEST(
    ShootFastComprisedCommand17MM,
    command_updates_agitator_setpoint_if_heat_limiting_disabled_and_within_tolerance)
{
    SETUP_TEST(ShootFastComprisedCommand17MM, false);
    RefSerial::RobotData robotData;
    robotData.turret.heat17ID1 = 10;
    robotData.turret.heatLimit17ID1 = 15;
    SETUP_EXPECTATIONS_NORMAL(0, true);
    RUN_TEST(false, 10, true);
}

TEST(
    ShootFastComprisedCommand17MM,
    command_updates_agitator_setpoint_if_heat_limiting_disabled_and_above_tolerance)
{
    SETUP_TEST(ShootFastComprisedCommand17MM, false);
    RefSerial::RobotData robotData;
    robotData.turret.heat17ID1 = 100;
    robotData.turret.heatLimit17ID1 = 95;
    SETUP_EXPECTATIONS_NORMAL(0, true);
    RUN_TEST(false, 10, true);
}

TEST(
    ShootFastComprisedCommand17MM,
    command_updates_agitator_setpoint_if_heat_ref_serial_disconnected_and_above_tolerance)
{
    SETUP_TEST(ShootFastComprisedCommand17MM, true);
    RefSerial::RobotData robotData;
    robotData.turret.heat17ID1 = 100;
    robotData.turret.heatLimit17ID1 = 95;
    SETUP_EXPECTATIONS_NORMAL(0, false);
    RUN_TEST(false, 10, true);
}
