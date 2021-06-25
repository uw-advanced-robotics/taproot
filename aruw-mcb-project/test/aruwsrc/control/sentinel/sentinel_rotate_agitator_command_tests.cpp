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

#include "aruwlib/architecture/clock.hpp"

#include "aruwsrc/control/sentinel/sentinel_rotate_agitator_command.hpp"
#include "aruwsrc/mock/agitator_subsystem_mock.hpp"
#include "aruwsrc/mock/sentinel_switcher_subsystem_mock.hpp"

using namespace aruwsrc::sentinel;
using namespace aruwlib::serial;
using namespace aruwlib;
using namespace testing;
using namespace aruwsrc::mock;
using namespace aruwlib::arch::clock;

#define SETUP_TEST()                                                    \
    Drivers drivers;                                                    \
    NiceMock<AgitatorSubsystemMock> agitator(&drivers);                 \
    NiceMock<SentinelSwitcherSubsystemMock> switcher(&drivers);         \
    SentinelRotateAgitatorCommand srac(&drivers, &agitator, &switcher); \
    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData()).WillRepeatedly(Return(true));

static void setHeatAndHeatLimit(
    RefSerial::RobotData &robotData,
    float heat17ID1,
    float heat17ID2,
    float heatLimit17ID1,
    float heatLimit17ID2)
{
    robotData.turret.heat17ID1 = heat17ID1;
    robotData.turret.heat17ID2 = heat17ID2;
    robotData.turret.heatLimit17ID1 = heatLimit17ID1;
    robotData.turret.heatLimit17ID2 = heatLimit17ID2;
}

TEST(SentinelRotateAgitatorCommand, isReady_returns_false_if_heat_limit_both_barrels_reached)
{
    Drivers drivers;
    NiceMock<AgitatorSubsystemMock> agitator(&drivers);
    NiceMock<SentinelSwitcherSubsystemMock> switcher(&drivers);
    SentinelRotateAgitatorCommand srac(&drivers, &agitator, &switcher);
    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData()).WillRepeatedly(Return(true));

    RefSerial::RobotData robotData;
    EXPECT_CALL(drivers.refSerial, getRobotData()).WillOnce(ReturnRef(robotData));

    setHeatAndHeatLimit(robotData, 100, 90, 95, 95);

    EXPECT_FALSE(srac.isReady());
}

TEST(
    SentinelRotateAgitatorCommand,
    isReady_returns_true_if_either_barrel_heat_limit_below_threshold)
{
    SETUP_TEST();

    RefSerial::RobotData robotData;
    EXPECT_CALL(drivers.refSerial, getRobotData()).Times(3).WillRepeatedly(ReturnRef(robotData));

    setHeatAndHeatLimit(robotData, 50, 100, 100, 140);
    EXPECT_TRUE(srac.isReady());
    setHeatAndHeatLimit(robotData, 100, 100, 100, 140);
    EXPECT_TRUE(srac.isReady());
    setHeatAndHeatLimit(robotData, 50, 135, 100, 140);
    EXPECT_TRUE(srac.isReady());
}

TEST(SentinelRotateAgitatorCommand, isReady_returns_true_if_ref_serial_offline)
{
    Drivers drivers;
    NiceMock<AgitatorSubsystemMock> agitator(&drivers);
    NiceMock<SentinelSwitcherSubsystemMock> switcher(&drivers);
    SentinelRotateAgitatorCommand srac(&drivers, &agitator, &switcher);
    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData()).WillRepeatedly(Return(false));

    EXPECT_TRUE(srac.isReady());
}

TEST(SentinelRotateAgitatorCommand, addCommand_to_scheduler_works_when_subsystems_scheduled)
{
    Drivers drivers;
    control::CommandScheduler scheduler(&drivers, true);
    NiceMock<AgitatorSubsystemMock> turret(&drivers);
    NiceMock<SentinelSwitcherSubsystemMock> switcher(&drivers);
    SentinelRotateAgitatorCommand srac(&drivers, &turret, &switcher);

    RefSerial::RobotData robotData;
    setHeatAndHeatLimit(robotData, 0, 0, 100, 100);
    EXPECT_CALL(drivers.refSerial, getRobotData()).WillRepeatedly(ReturnRef(robotData));

    EXPECT_CALL(drivers.errorController, addToErrorList);
    scheduler.addCommand(&srac);

    EXPECT_FALSE(scheduler.isCommandScheduled(&srac));

    scheduler.registerSubsystem(&turret);
    scheduler.registerSubsystem(&switcher);

    scheduler.addCommand(&srac);

    EXPECT_TRUE(scheduler.isCommandScheduled(&srac));
}

TEST(SentinelRotateAgitatorCommand, initialize_when_not_overheated_starts_rotating_agitator)
{
    SETUP_TEST();

    RefSerial::RobotData robotData;
    setHeatAndHeatLimit(robotData, 0, 0, 100, 100);
    EXPECT_CALL(drivers.refSerial, getRobotData()).WillRepeatedly(ReturnRef(robotData));

    srac.initialize();

    EXPECT_CALL(agitator, setSetpoint);
    ON_CALL(agitator, isOnline).WillByDefault(Return(true));
    ON_CALL(agitator, isJammed).WillByDefault(Return(false));
    srac.execute();
}

TEST(SentinelRotateAgitatorCommand, initialize_when_overheated_switches_from_lower_to_upper_barrel)
{
    SETUP_TEST();

    RefSerial::RobotData robotData;
    setHeatAndHeatLimit(robotData, 100, 0, 100, 100);
    EXPECT_CALL(drivers.refSerial, getRobotData()).WillRepeatedly(ReturnRef(robotData));

    EXPECT_CALL(switcher, isLowerUsed).WillRepeatedly(Return(true));
    EXPECT_CALL(switcher, useLowerBarrel(false));
    srac.initialize();
}

TEST(SentinelRotateAgitatorCommand, initialize_when_overheated_switches_from_upper_to_lower_barrel)
{
    SETUP_TEST();

    RefSerial::RobotData robotData;
    setHeatAndHeatLimit(robotData, 0, 100, 100, 100);
    EXPECT_CALL(drivers.refSerial, getRobotData()).WillRepeatedly(ReturnRef(robotData));

    EXPECT_CALL(switcher, isLowerUsed).WillRepeatedly(Return(false));
    EXPECT_CALL(switcher, useLowerBarrel(true));
    srac.initialize();
}

TEST(SentinelRotateAgitatorCommand, initialize_when_overheated_waits_before_rotating_agitator)
{
    setTime(0);
    SETUP_TEST();

    RefSerial::RobotData robotData;
    setHeatAndHeatLimit(robotData, 0, 100, 100, 100);
    EXPECT_CALL(drivers.refSerial, getRobotData()).WillRepeatedly(ReturnRef(robotData));

    EXPECT_CALL(switcher, isLowerUsed).WillRepeatedly(Return(false));
    EXPECT_CALL(switcher, useLowerBarrel(true));
    ON_CALL(agitator, isOnline).WillByDefault(Return(true));
    ON_CALL(agitator, isJammed).WillByDefault(Return(false));

    srac.initialize();

    // Won't call setAgitatorDesiredAngle until some time has passed
    srac.execute();

    EXPECT_CALL(agitator, setSetpoint);
    setTime(10000);
    srac.execute();
}

TEST(
    SentinelRotateAgitatorCommand,
    isFinished_returns_false_when_not_overheating_and_agitator_not_within_setpoint)
{
    SETUP_TEST();

    RefSerial::RobotData robotData;
    setHeatAndHeatLimit(robotData, 0, 0, 100, 100);
    EXPECT_CALL(drivers.refSerial, getRobotData()).WillRepeatedly(ReturnRef(robotData));

    srac.initialize();
    EXPECT_FALSE(srac.isFinished());
}

TEST(
    SentinelRotateAgitatorCommand,
    isFinished_returns_false_when_overheating_and_switching_switcher)
{
    SETUP_TEST();

    RefSerial::RobotData robotData;
    setHeatAndHeatLimit(robotData, 100, 100, 100, 100);
    EXPECT_CALL(drivers.refSerial, getRobotData()).WillRepeatedly(ReturnRef(robotData));

    srac.initialize();
    EXPECT_FALSE(srac.isFinished());
}

TEST(
    SentinelRotateAgitatorCommand,
    isFinished_returns_true_when_not_overheating_and_agitator_rotation_complete)
{
    setTime(0);
    SETUP_TEST();

    RefSerial::RobotData robotData;
    setHeatAndHeatLimit(robotData, 0, 0, 100, 100);
    EXPECT_CALL(drivers.refSerial, getRobotData()).WillRepeatedly(ReturnRef(robotData));

    ON_CALL(agitator, getCurrentValue).WillByDefault(Return(10));
    ON_CALL(agitator, getSetpoint).WillByDefault(Return(10));

    srac.initialize();
    setTime(10000);
    srac.execute();
    EXPECT_TRUE(srac.isFinished());
}
