/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/integrable_setpoint_subsystem_mock.hpp"

using namespace testing;
using namespace tap::control::setpoint;

class MoveIntegralCommandTest : public Test
{
protected:
    MoveIntegralCommandTest() : sub(&drivers) {}

    void SetUp() override
    {
        ON_CALL(sub, isOnline).WillByDefault(ReturnPointee(&online));
        ON_CALL(sub, isJammed).WillByDefault(ReturnPointee(&jammed));
        ON_CALL(sub, getCurrentValueIntegral).WillByDefault(ReturnPointee(&integral));
    }

    tap::Drivers drivers;
    testing::NiceMock<tap::mock::IntegrableSetpointSubsystemMock> sub;

    bool online = false;
    bool jammed = false;
    float integral = 0;

    MoveIntegralCommand::Config defaultConfig = {
        .targetIntegralChange = 5,
        .desiredSetpoint = 10,
        .integralSetpointTolerance = 1,
    };
};

class MoveIntegralCommandTestP : public MoveIntegralCommandTest,
                                 public WithParamInterface<MoveIntegralCommand::Config>
{
protected:
    void SetUp() override { cmd = new MoveIntegralCommand(sub, GetParam()); }

    MoveIntegralCommand *cmd;
};

TEST_F(MoveIntegralCommandTest, isReady_when_jammed)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = true;
    online = true;

    EXPECT_FALSE(cmd.isReady());
}

TEST_F(MoveIntegralCommandTest, isReady_when_not_jammed)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = false;
    online = true;

    EXPECT_TRUE(cmd.isReady());
}

TEST_F(MoveIntegralCommandTest, isReady_when_sub_offline)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = false;
    online = false;

    EXPECT_FALSE(cmd.isReady());
}

TEST_F(MoveIntegralCommandTest, isFinished_when_sub_offline)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = false;
    online = false;

    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(MoveIntegralCommandTest, isFinished_when_sub_online)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    ON_CALL(sub, getCurrentValueIntegral).WillByDefault(Return(0));

    jammed = false;
    online = true;

    cmd.initialize();
    EXPECT_FALSE(cmd.isFinished());
}

TEST_F(MoveIntegralCommandTest, isFinished_when_jammed)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = true;
    online = false;

    cmd.initialize();
    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(MoveIntegralCommandTest, isFinished_when_integral_past_setpoint)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = false;
    online = true;

    cmd.initialize();

    integral = defaultConfig.targetIntegralChange;

    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(MoveIntegralCommandTest, isFinished_when_integral_past_setpoint_negative_direction)
{
    defaultConfig.desiredSetpoint = -defaultConfig.desiredSetpoint;
    defaultConfig.targetIntegralChange = -defaultConfig.targetIntegralChange;
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = false;
    online = true;

    cmd.initialize();

    integral = defaultConfig.targetIntegralChange;

    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(MoveIntegralCommandTest, isFinished_when_integral_far_past_setpoint)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = false;
    online = true;

    cmd.initialize();

    integral =
        defaultConfig.targetIntegralChange + defaultConfig.integralSetpointTolerance * 100.0f;

    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(MoveIntegralCommandTest, end_sets_output_to_0)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = false;
    online = true;

    EXPECT_CALL(sub, setSetpoint(0)).Times(2);

    cmd.end(false);
    cmd.end(true);
}

TEST_F(MoveIntegralCommandTest, initialize_sets_setpoint_to_desired_setpoint)
{
    MoveIntegralCommand cmd(sub, defaultConfig);

    jammed = false;
    online = true;

    EXPECT_CALL(sub, setSetpoint(defaultConfig.desiredSetpoint));

    cmd.initialize();
}
