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

#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/integrable_setpoint_subsystem_mock.hpp"

using namespace testing;
using namespace tap::control::setpoint;

class UnjamIntegralCommandTest : public Test
{
protected:
    UnjamIntegralCommandTest() : sub(&drivers) {}

    void SetUp() override
    {
        ON_CALL(sub, isOnline).WillByDefault(ReturnPointee(&online));
        ON_CALL(sub, isJammed).WillByDefault(ReturnPointee(&jammed));
        ON_CALL(sub, getCurrentValueIntegral).WillByDefault(ReturnPointee(&integral));
        ON_CALL(sub, setSetpoint).WillByDefault([&](float setpoint) { this->setpoint = setpoint; });
        ON_CALL(sub, setDesiredIntegralSetpoint).WillByDefault([&](float integralSetpoint) {
            this->integralSetpoint = integralSetpoint;
        });
        ON_CALL(sub, getDesiredIntegralSetpoint).WillByDefault(ReturnPointee(&integralSetpoint));
    }

    void clearJam(UnjamIntegralCommand &cmd)
    {
        online = true;
        jammed = false;

        cmd.initialize();

        EXPECT_NEAR(-defaultConfig.unjamSetpoint, setpoint, 1E-5);

        cmd.execute();  // integral value hasn't changed

        EXPECT_NEAR(-defaultConfig.unjamSetpoint, setpoint, 1E-5);

        integral -= defaultConfig.targetUnjamIntegralChange;

        cmd.execute();  // in unjam forwards state

        EXPECT_NEAR(defaultConfig.unjamSetpoint, setpoint, 1E-5);

        integral += defaultConfig.targetUnjamIntegralChange;

        cmd.execute();  // unjam complete

        EXPECT_TRUE(cmd.isFinished());
    }

    tap::Drivers drivers;
    testing::NiceMock<tap::mock::IntegrableSetpointSubsystemMock> sub;

    bool online = false;
    bool jammed = false;
    float integral = 0;
    float setpoint = 0;
    float integralSetpoint = 0;

    UnjamIntegralCommand::Config defaultConfig = {
        .targetUnjamIntegralChange = 10,
        .unjamSetpoint = 2,
        .extraWaitTime = 0,
        .targetCycleCount = 1,
    };
};

TEST_F(UnjamIntegralCommandTest, isReady_motor_offline)
{
    UnjamIntegralCommand cmd(sub, defaultConfig);

    online = false;

    EXPECT_FALSE(cmd.isReady());
}

TEST_F(UnjamIntegralCommandTest, isReady_motor_online)
{
    UnjamIntegralCommand cmd(sub, defaultConfig);

    online = true;

    EXPECT_TRUE(cmd.isReady());
}

TEST_F(UnjamIntegralCommandTest, isFinished_motor_offline)
{
    UnjamIntegralCommand cmd(sub, defaultConfig);

    online = false;

    cmd.initialize();
    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(UnjamIntegralCommandTest, isFinished_motor_online)
{
    UnjamIntegralCommand cmd(sub, defaultConfig);

    online = true;

    cmd.initialize();
    EXPECT_FALSE(cmd.isFinished());
}

TEST_F(UnjamIntegralCommandTest, execute_jam_cleared_immediately)
{
    UnjamIntegralCommand cmd(sub, defaultConfig);

    clearJam(cmd);
}

TEST_F(UnjamIntegralCommandTest, execute_jam_clear_timeout)
{
    tap::arch::clock::ClockStub clock;

    UnjamIntegralCommand cmd(sub, defaultConfig);

    online = true;
    jammed = false;

    cmd.initialize();

    EXPECT_NEAR(-defaultConfig.unjamSetpoint, setpoint, 1E-5);

    clock.time +=
        1000 * defaultConfig.targetUnjamIntegralChange / defaultConfig.unjamSetpoint + 100;

    cmd.execute();  // integral hasn't changed, but timed out

    EXPECT_NEAR(defaultConfig.unjamSetpoint, setpoint, 1E-5);

    EXPECT_FALSE(cmd.isFinished());

    clock.time +=
        1000 * defaultConfig.targetUnjamIntegralChange / defaultConfig.unjamSetpoint + 100;

    cmd.execute();  // integral hasn't changed but we have reached the forward position (since we
                    // didn't move); however, we set targetCycleCount to 1, so the command finishes
                    // anyway.

    EXPECT_NEAR(-defaultConfig.unjamSetpoint, setpoint, 1E-5);

    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(UnjamIntegralCommandTest, execute_jam_stuck_in_back_state)
{
    tap::arch::clock::ClockStub clock;

    UnjamIntegralCommand cmd(sub, defaultConfig);

    online = true;
    jammed = false;

    cmd.initialize();

    EXPECT_NEAR(-defaultConfig.unjamSetpoint, setpoint, 1E-5);

    integral -= defaultConfig.targetUnjamIntegralChange;

    cmd.execute();  // integral changed, in unjam forward state

    EXPECT_NEAR(defaultConfig.unjamSetpoint, setpoint, 1E-5);

    clock.time +=
        1000 * defaultConfig.targetUnjamIntegralChange / defaultConfig.unjamSetpoint + 100;

    cmd.execute();  // still in back position timed out

    EXPECT_NEAR(-defaultConfig.unjamSetpoint, setpoint, 1E-5);

    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(UnjamIntegralCommandTest, execute_multiple_jam_iterations)
{
    defaultConfig.targetCycleCount = 5;
    tap::arch::clock::ClockStub clock;

    UnjamIntegralCommand cmd(sub, defaultConfig);

    online = true;
    jammed = false;

    cmd.initialize();

    EXPECT_NEAR(-defaultConfig.unjamSetpoint, setpoint, 1E-5);

    for (size_t i = 0; i < 5; i++)
    {
        EXPECT_FALSE(cmd.isFinished());

        clock.time +=
            1000 * defaultConfig.targetUnjamIntegralChange / defaultConfig.unjamSetpoint + 100;

        cmd.execute();

        EXPECT_NEAR(defaultConfig.unjamSetpoint, setpoint, 1E-5);

        clock.time +=
            1000 * defaultConfig.targetUnjamIntegralChange / defaultConfig.unjamSetpoint + 100;

        cmd.execute();

        EXPECT_NEAR(-defaultConfig.unjamSetpoint, setpoint, 1E-5);
    }

    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(UnjamIntegralCommandTest, end_sets_setpoint_to_0)
{
    UnjamIntegralCommand cmd(sub, defaultConfig);

    cmd.initialize();

    cmd.end(false);
    EXPECT_NEAR(0, setpoint, 1E-5);
    cmd.end(true);
    EXPECT_NEAR(0, setpoint, 1E-5);
}

TEST_F(UnjamIntegralCommandTest, end_clears_jam_when_jam_cleared)
{
    UnjamIntegralCommand cmd(sub, defaultConfig);

    EXPECT_CALL(sub, clearJam);

    clearJam(cmd);

    cmd.end(false);
}
