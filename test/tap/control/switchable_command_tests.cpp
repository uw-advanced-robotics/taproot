/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/control/switchable_command.hpp"
#include "tap/drivers.hpp"

#include "test_subsystem.hpp"

using namespace tap::control;
using tap::Drivers;

namespace
{
class RecordingCommand : public Command
{
public:
    explicit RecordingCommand(Subsystem* subsystem) { addSubsystemRequirement(subsystem); }

    bool isReady() override { return ready; }
    void initialize() override { ++initializeCount; }
    void execute() override { ++executeCount; }
    void end(bool wasInterrupted) override
    {
        ++endCount;
        interrupted = wasInterrupted;
    }
    bool isFinished() const override { return false; }
    const char* getName() const override { return "recording command"; }

    bool ready = true;
    bool interrupted = false;
    int initializeCount = 0;
    int executeCount = 0;
    int endCount = 0;
};
}  // namespace

TEST(SwitchableCommand, runs_the_selected_command)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem onTrueSubsystem(&drivers);
    TestSubsystem onFalseSubsystem(&drivers);
    scheduler.registerSubsystem(&onTrueSubsystem);
    scheduler.registerSubsystem(&onFalseSubsystem);

    bool selectTrue = false;
    RecordingCommand onTrue(&onTrueSubsystem);
    RecordingCommand onFalse(&onFalseSubsystem);
    SwitchableCommand command(&onTrue, &onFalse, [&selectTrue]() { return selectTrue; });

    EXPECT_EQ(
        command.getRequirementsBitwise(),
        onTrue.getRequirementsBitwise() | onFalse.getRequirementsBitwise());
    EXPECT_FALSE(command.isFinished());

    scheduler.addCommand(&command);
    scheduler.run();
    EXPECT_EQ(onFalse.initializeCount, 1);
    EXPECT_EQ(onFalse.executeCount, 1);
    EXPECT_EQ(onTrue.executeCount, 0);

    selectTrue = true;
    scheduler.run();
    EXPECT_EQ(onFalse.endCount, 1);
    EXPECT_TRUE(onFalse.interrupted);
    EXPECT_EQ(onTrue.initializeCount, 1);
    EXPECT_EQ(onTrue.executeCount, 1);

    selectTrue = false;
    scheduler.run();
    EXPECT_EQ(onTrue.endCount, 1);
    EXPECT_EQ(onFalse.initializeCount, 2);
    EXPECT_EQ(onFalse.executeCount, 2);
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));
}

TEST(SwitchableCommand, readiness_follows_the_selected_command)
{
    Drivers drivers;
    TestSubsystem onTrueSubsystem(&drivers);
    TestSubsystem onFalseSubsystem(&drivers);
    bool selectTrue = true;
    RecordingCommand onTrue(&onTrueSubsystem);
    RecordingCommand onFalse(&onFalseSubsystem);
    onTrue.ready = false;
    onFalse.ready = true;
    SwitchableCommand command(&onTrue, &onFalse, [&selectTrue]() { return selectTrue; });

    EXPECT_FALSE(command.isReady());
    selectTrue = false;
    EXPECT_TRUE(command.isReady());
}

TEST(SwitchableCommand, end_ends_only_the_active_command)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem onTrueSubsystem(&drivers);
    TestSubsystem onFalseSubsystem(&drivers);
    scheduler.registerSubsystem(&onTrueSubsystem);
    scheduler.registerSubsystem(&onFalseSubsystem);

    RecordingCommand onTrue(&onTrueSubsystem);
    RecordingCommand onFalse(&onFalseSubsystem);
    SwitchableCommand command(&onTrue, &onFalse, []() { return true; });

    scheduler.addCommand(&command);
    scheduler.removeCommand(&command, true);
    EXPECT_EQ(onTrue.endCount, 1);
    EXPECT_TRUE(onTrue.interrupted);
    EXPECT_EQ(onFalse.endCount, 0);
}
