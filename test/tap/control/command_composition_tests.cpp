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

#include <memory>

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "test_subsystem.hpp"

using namespace tap::control;
using tap::Drivers;
using tap::arch::clock::ClockStub;

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
    bool isFinished() const override { return finished; }
    const char* getName() const override { return "recording command"; }

    bool finished = false;
    bool ready = true;
    bool interrupted = false;
    int initializeCount = 0;
    int executeCount = 0;
    int endCount = 0;
};

subsystem_scheduler_bitmap_t requirementBit(const Subsystem* subsystem)
{
    return static_cast<subsystem_scheduler_bitmap_t>(1) << subsystem->getGlobalIdentifier();
}
}  // namespace

TEST(CommandComposition, andThen_runs_commands_in_order)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem firstSubsystem(&drivers);
    TestSubsystem secondSubsystem(&drivers);
    TestSubsystem thirdSubsystem(&drivers);
    scheduler.registerSubsystem(&firstSubsystem);
    scheduler.registerSubsystem(&secondSubsystem);
    scheduler.registerSubsystem(&thirdSubsystem);

    RecordingCommand first(&firstSubsystem);
    RecordingCommand second(&secondSubsystem);
    RecordingCommand third(&thirdSubsystem);
    first.finished = true;
    second.finished = true;
    third.finished = true;
    std::unique_ptr<Command> command(first.andThen({&second, &third}));

    scheduler.addCommand(command.get());
    scheduler.run();
    EXPECT_EQ(first.endCount, 1);
    EXPECT_EQ(second.initializeCount, 0);

    scheduler.run();
    EXPECT_EQ(second.endCount, 1);
    EXPECT_EQ(third.initializeCount, 0);

    scheduler.run();
    EXPECT_EQ(third.endCount, 1);
    EXPECT_FALSE(first.interrupted);
    EXPECT_FALSE(scheduler.isCommandScheduled(command.get()));
}

TEST(CommandComposition, alongWith_runs_commands_until_all_finish)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem firstSubsystem(&drivers);
    TestSubsystem secondSubsystem(&drivers);
    scheduler.registerSubsystem(&firstSubsystem);
    scheduler.registerSubsystem(&secondSubsystem);

    RecordingCommand first(&firstSubsystem);
    RecordingCommand second(&secondSubsystem);
    first.finished = true;
    std::unique_ptr<Command> command(first.alongWith(&second));

    scheduler.addCommand(command.get());
    EXPECT_EQ(first.initializeCount, 1);
    EXPECT_EQ(second.initializeCount, 1);

    scheduler.run();
    EXPECT_EQ(first.executeCount, 1);
    EXPECT_EQ(second.executeCount, 1);
    EXPECT_TRUE(scheduler.isCommandScheduled(command.get()));

    scheduler.run();
    EXPECT_EQ(first.executeCount, 1);
    EXPECT_EQ(second.executeCount, 2);

    second.finished = true;
    scheduler.run();
    EXPECT_FALSE(scheduler.isCommandScheduled(command.get()));
}

TEST(CommandComposition, until_ends_when_condition_becomes_true)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem subsystem(&drivers);
    scheduler.registerSubsystem(&subsystem);

    bool done = false;
    RecordingCommand command(&subsystem);
    std::unique_ptr<Command> until(command.until([&done]() { return done; }));

    scheduler.addCommand(until.get());
    scheduler.run();
    EXPECT_EQ(command.executeCount, 1);
    EXPECT_TRUE(scheduler.isCommandScheduled(until.get()));

    done = true;
    scheduler.run();
    EXPECT_EQ(command.endCount, 1);
    EXPECT_TRUE(command.interrupted);
    EXPECT_FALSE(scheduler.isCommandScheduled(until.get()));
}

TEST(CommandComposition, onlyWhile_ends_when_condition_becomes_false)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem subsystem(&drivers);
    scheduler.registerSubsystem(&subsystem);

    bool keepRunning = true;
    RecordingCommand command(&subsystem);
    std::unique_ptr<Command> onlyWhile(command.onlyWhile([&keepRunning]() { return keepRunning; }));

    scheduler.addCommand(onlyWhile.get());
    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(onlyWhile.get()));

    keepRunning = false;
    scheduler.run();
    EXPECT_EQ(command.endCount, 1);
    EXPECT_TRUE(command.interrupted);
    EXPECT_FALSE(scheduler.isCommandScheduled(onlyWhile.get()));
}

TEST(CommandComposition, withTimeout_ends_after_the_timeout)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem subsystem(&drivers);
    scheduler.registerSubsystem(&subsystem);
    ClockStub clock;

    RecordingCommand command(&subsystem);
    std::unique_ptr<Command> timeout(command.withTimeout(10));
    scheduler.addCommand(timeout.get());

    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(timeout.get()));

    clock.time = 11;
    scheduler.run();
    EXPECT_EQ(command.endCount, 1);
    EXPECT_TRUE(command.interrupted);
    EXPECT_FALSE(scheduler.isCommandScheduled(timeout.get()));
}

TEST(CommandComposition, deadlineWith_ends_when_the_deadline_finishes)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem subsystem(&drivers);
    TestSubsystem deadlineSubsystem(&drivers);
    scheduler.registerSubsystem(&subsystem);
    scheduler.registerSubsystem(&deadlineSubsystem);

    RecordingCommand command(&subsystem);
    RecordingCommand deadline(&deadlineSubsystem);
    std::unique_ptr<Command> group(command.deadlineWith(&deadline));

    scheduler.addCommand(group.get());
    scheduler.run();
    EXPECT_EQ(command.executeCount, 1);
    EXPECT_TRUE(scheduler.isCommandScheduled(group.get()));

    deadline.finished = true;
    scheduler.run();
    EXPECT_EQ(command.endCount, 1);
    EXPECT_GE(deadline.endCount, 1);
    EXPECT_FALSE(scheduler.isCommandScheduled(group.get()));
}

TEST(CommandComposition, sequence_and_parallel_run_the_given_commands)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem firstSubsystem(&drivers);
    TestSubsystem secondSubsystem(&drivers);
    scheduler.registerSubsystem(&firstSubsystem);
    scheduler.registerSubsystem(&secondSubsystem);

    RecordingCommand first(&firstSubsystem);
    RecordingCommand second(&secondSubsystem);
    first.finished = true;
    second.finished = true;

    std::unique_ptr<Command> sequence(first.sequence({&second}));
    scheduler.addCommand(sequence.get());
    scheduler.run();
    scheduler.run();
    EXPECT_EQ(first.endCount, 1);
    EXPECT_EQ(second.endCount, 1);
    EXPECT_FALSE(scheduler.isCommandScheduled(sequence.get()));

    first.finished = true;
    second.finished = true;
    first.executeCount = 0;
    second.executeCount = 0;
    std::unique_ptr<Command> parallel(first.parallel({&second}));
    EXPECT_TRUE(parallel->isReady());
    second.ready = false;
    EXPECT_TRUE(parallel->isReady());

    scheduler.addCommand(parallel.get());
    scheduler.run();
    EXPECT_EQ(first.executeCount, 1);
    EXPECT_EQ(second.executeCount, 1);
    EXPECT_FALSE(scheduler.isCommandScheduled(parallel.get()));
}

TEST(CommandComposition, until_is_not_ready_when_the_command_is_not_ready)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem subsystem(&drivers);
    scheduler.registerSubsystem(&subsystem);

    RecordingCommand command(&subsystem);
    command.ready = false;
    std::unique_ptr<Command> until(command.until([]() { return false; }));

    EXPECT_FALSE(until->isReady());
    scheduler.addCommand(until.get());
    EXPECT_FALSE(scheduler.isCommandScheduled(until.get()));
}

TEST(CommandComposition, runOnce_and_run_require_the_given_subsystems)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem subsystem(&drivers);
    scheduler.registerSubsystem(&subsystem);

    int runs = 0;
    std::unique_ptr<Command> once(Command::runOnce([&runs]() { ++runs; }, {&subsystem}));
    EXPECT_EQ(once->getRequirementsBitwise(), requirementBit(&subsystem));
    scheduler.addCommand(once.get());
    EXPECT_EQ(runs, 1);
    scheduler.run();
    EXPECT_FALSE(scheduler.isCommandScheduled(once.get()));

    std::unique_ptr<Command> run(Command::run([&runs]() { ++runs; }, {&subsystem}));
    EXPECT_EQ(run->getRequirementsBitwise(), requirementBit(&subsystem));
    scheduler.addCommand(run.get());
    scheduler.run();
    scheduler.run();
    EXPECT_EQ(runs, 3);
    EXPECT_TRUE(scheduler.isCommandScheduled(run.get()));
    EXPECT_FALSE(run->isFinished());
}

TEST(CommandComposition, either_runs_the_branch_selected_by_the_condition)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem onTrueSubsystem(&drivers);
    TestSubsystem onFalseSubsystem(&drivers);
    scheduler.registerSubsystem(&onTrueSubsystem);
    scheduler.registerSubsystem(&onFalseSubsystem);

    bool selectTrue = true;
    RecordingCommand onTrue(&onTrueSubsystem);
    RecordingCommand onFalse(&onFalseSubsystem);
    std::unique_ptr<Command> command(
        Command::either(&onTrue, &onFalse, [&selectTrue]() { return selectTrue; }));

    scheduler.addCommand(command.get());
    scheduler.run();
    EXPECT_EQ(onTrue.executeCount, 1);
    EXPECT_EQ(onFalse.executeCount, 0);

    selectTrue = false;
    scheduler.run();
    EXPECT_EQ(onTrue.endCount, 1);
    EXPECT_EQ(onFalse.executeCount, 1);
}

TEST(CommandComposition, subsystem_runOnce_and_run_require_that_subsystem)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem subsystem(&drivers);
    TestSubsystem other(&drivers);
    scheduler.registerSubsystem(&subsystem);
    scheduler.registerSubsystem(&other);

    int runs = 0;
    std::unique_ptr<Command> once(subsystem.runOnce([&runs]() { ++runs; }, {&other}));
    EXPECT_EQ(once->getRequirementsBitwise(), requirementBit(&subsystem) | requirementBit(&other));
    scheduler.addCommand(once.get());
    EXPECT_EQ(runs, 1);
    scheduler.run();
    EXPECT_FALSE(scheduler.isCommandScheduled(once.get()));

    std::unique_ptr<Command> run(subsystem.run([&runs]() { ++runs; }));
    EXPECT_EQ(run->getRequirementsBitwise(), requirementBit(&subsystem));
    scheduler.addCommand(run.get());
    scheduler.run();
    EXPECT_EQ(runs, 2);
    EXPECT_TRUE(scheduler.isCommandScheduled(run.get()));
    EXPECT_FALSE(run->isFinished());
}
