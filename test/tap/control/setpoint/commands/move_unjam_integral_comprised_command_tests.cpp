

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

#include <memory>

#include <gtest/gtest.h>

#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/integrable_setpoint_subsystem_mock.hpp"
#include "tap/mock/move_integral_command_mock.hpp"
#include "tap/mock/unjam_integral_command_mock.hpp"

using namespace testing;
using namespace tap::control::setpoint;

class MoveUnjamIntegralComprisedCommandTest : public Test
{
protected:
    MoveUnjamIntegralComprisedCommandTest()
        : sub(&drivers),
          moveIntegralCommand(sub),
          unjamIntegralCommand(sub),
          cmd(nullptr)
    {
    }

    void SetUp() override
    {
        ON_CALL(moveIntegralCommand, getRequirementsBitwise)
            .WillByDefault(Return(1UL << sub.getGlobalIdentifier()));
        ON_CALL(unjamIntegralCommand, getRequirementsBitwise)
            .WillByDefault(Return(1UL << sub.getGlobalIdentifier()));

        cmd = std::unique_ptr<MoveUnjamIntegralComprisedCommand>(
            new MoveUnjamIntegralComprisedCommand(
                drivers,
                sub,
                moveIntegralCommand,
                unjamIntegralCommand));

        ON_CALL(sub, isJammed).WillByDefault(ReturnPointee(&jammed));
    }

    tap::Drivers drivers;
    testing::NiceMock<tap::mock::IntegrableSetpointSubsystemMock> sub;
    testing::NiceMock<tap::mock::MoveIntegralCommandMock> moveIntegralCommand;
    testing::NiceMock<tap::mock::UnjamIntegralCommandMock> unjamIntegralCommand;
    std::unique_ptr<MoveUnjamIntegralComprisedCommand> cmd;

    bool jammed = false;
};

TEST_F(MoveUnjamIntegralComprisedCommandTest, isReady_not_jammed)
{
    jammed = false;
    bool ready = false;

    ON_CALL(moveIntegralCommand, isReady).WillByDefault(ReturnPointee(&ready));

    EXPECT_FALSE(cmd->isReady());

    ready = true;

    EXPECT_TRUE(cmd->isReady());
}

TEST_F(MoveUnjamIntegralComprisedCommandTest, isReady_jammed)
{
    jammed = true;
    bool ready = false;

    ON_CALL(unjamIntegralCommand, isReady).WillByDefault(ReturnPointee(&ready));

    EXPECT_FALSE(cmd->isReady());

    ready = true;

    EXPECT_TRUE(cmd->isReady());
}

TEST_F(MoveUnjamIntegralComprisedCommandTest, initialize_adds_move_command)
{
    jammed = false;

    ON_CALL(moveIntegralCommand, isReady).WillByDefault(Return(true));
    EXPECT_CALL(moveIntegralCommand, initialize);

    cmd->initialize();
}

TEST_F(MoveUnjamIntegralComprisedCommandTest, execute_not_jammed_runs_move_command_until_completion)
{
    jammed = false;

    bool finished = false;

    ON_CALL(moveIntegralCommand, isReady).WillByDefault(Return(true));
    ON_CALL(moveIntegralCommand, isFinished).WillByDefault(ReturnPointee(&finished));
    EXPECT_CALL(moveIntegralCommand, initialize);
    EXPECT_CALL(moveIntegralCommand, execute);

    cmd->initialize();

    finished = true;

    cmd->execute();

    EXPECT_TRUE(cmd->isFinished());
}

TEST_F(MoveUnjamIntegralComprisedCommandTest, execute_jammed_runs_move_unjam_until_completion)
{
    jammed = true;

    bool finished = false;

    ON_CALL(unjamIntegralCommand, isReady).WillByDefault(Return(true));
    ON_CALL(unjamIntegralCommand, isFinished).WillByDefault(ReturnPointee(&finished));
    ON_CALL(moveIntegralCommand, isReady).WillByDefault(Return(true));
    ON_CALL(moveIntegralCommand, isFinished).WillByDefault(Return(true));
    EXPECT_CALL(unjamIntegralCommand, initialize);
    EXPECT_CALL(unjamIntegralCommand, execute);

    cmd->initialize();

    finished = true;

    cmd->execute();

    EXPECT_TRUE(cmd->isFinished());
}

TEST_F(MoveUnjamIntegralComprisedCommandTest, end_ends_move_command_not_jammed)
{
    jammed = false;

    ON_CALL(moveIntegralCommand, isReady).WillByDefault(Return(true));
    EXPECT_CALL(moveIntegralCommand, end);

    cmd->initialize();

    cmd->end(true);
}

TEST_F(MoveUnjamIntegralComprisedCommandTest, end_ends_unjam_command_jammed)
{
    jammed = true;

    ON_CALL(unjamIntegralCommand, isReady).WillByDefault(Return(true));
    EXPECT_CALL(unjamIntegralCommand, end);

    cmd->execute();

    cmd->end(true);
}
