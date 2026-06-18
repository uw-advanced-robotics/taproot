/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/control/concurrent_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/command_mock.hpp"

#include "test_command.hpp"
#include "test_subsystem.hpp"

using namespace tap::control;
using std::set;
using tap::Drivers;
using tap::mock::CommandMock;
using namespace testing;

static subsystem_scheduler_bitmap_t calcRequirementsBitwise(const set<Subsystem *> subRequirements)
{
    subsystem_scheduler_bitmap_t sum = 0;
    for (const auto sub : subRequirements)
    {
        sum |= (static_cast<subsystem_scheduler_bitmap_t>(1) << sub->getGlobalIdentifier());
    }
    return sum;
}

TEST(ConcurrentCommands, one_command_is_run)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);

    NiceMock<CommandMock> c1;

    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));
    std::array<std::pair<Command *, bool>, 1> commands = {{{&c1, false}}};
    ConcurrentCommand<1> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);
    scheduler.run();

    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, two_commands_are_run)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    NiceMock<CommandMock> c1;
    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    TestSubsystem s2(&drivers);
    scheduler.registerSubsystem(&s2);
    NiceMock<CommandMock> c2;
    requirements = {&s2};
    EXPECT_CALL(c2, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    std::array<std::pair<Command *, bool>, 2> commands = {{{&c1, false}, {&c2, false}}};
    ConcurrentCommand<2> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    EXPECT_CALL(c2, isReady).WillOnce(Return(true));
    EXPECT_CALL(c2, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);
    EXPECT_CALL(c2, execute).Times(1);
    EXPECT_CALL(c2, isFinished).WillOnce(Return(true));
    EXPECT_CALL(c2, end(false)).Times(1);
    scheduler.run();

    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, two_commands_are_run_until_finished)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    NiceMock<CommandMock> c1;
    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    TestSubsystem s2(&drivers);
    scheduler.registerSubsystem(&s2);
    NiceMock<CommandMock> c2;
    requirements = {&s2};
    EXPECT_CALL(c2, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    std::array<std::pair<Command *, bool>, 2> commands = {{{&c1, false}, {&c2, false}}};
    ConcurrentCommand<2> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    EXPECT_CALL(c2, isReady).WillOnce(Return(true));
    EXPECT_CALL(c2, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);
    EXPECT_CALL(c2, execute).Times(2);
    EXPECT_CALL(c2, isFinished).WillOnce(Return(false)).WillOnce(Return(true));
    EXPECT_CALL(c2, end(false)).Times(1);

    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    scheduler.run();
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, racing_two_commands_finishes_with_one)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    NiceMock<CommandMock> c1;
    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    TestSubsystem s2(&drivers);
    scheduler.registerSubsystem(&s2);
    NiceMock<CommandMock> c2;
    requirements = {&s2};
    EXPECT_CALL(c2, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    std::array<std::pair<Command *, bool>, 2> commands = {{{&c1, false}, {&c2, false}}};
    ConcurrentRaceCommand<2> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    EXPECT_CALL(c2, isReady).WillOnce(Return(true));
    EXPECT_CALL(c2, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(2);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(false)).WillOnce(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);
    EXPECT_CALL(c2, execute).Times(2);
    EXPECT_CALL(c2, isFinished).Times(2).WillRepeatedly(Return(false));
    EXPECT_CALL(c2, end(true)).Times(1);

    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    scheduler.run();
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, not_added_when_not_ready)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);

    NiceMock<CommandMock> c1;

    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));
    std::array<std::pair<Command *, bool>, 1> commands = {{{&c1, false}}};
    ConcurrentCommand<1> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(false));
    scheduler.addCommand(&command);
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, cancelling_command_ends_internal_commands)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);

    NiceMock<CommandMock> c1;

    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));
    std::array<std::pair<Command *, bool>, 1> commands = {{{&c1, false}}};
    ConcurrentCommand<1> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(false));
    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    EXPECT_CALL(c1, end(true)).Times(1);
    scheduler.removeCommand(&command, true);
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, null_command_asserts_DEATH)
{
    std::array<std::pair<Command *, bool>, 1> commands = {{{nullptr, false}}};
    ASSERT_DEATH({ ConcurrentCommand<1> command(commands, "test command"); }, ".*");
}

TEST(ConcurrentCommands, overlapping_requirements_asserts_DEATH)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    TestCommand c1(&s1);
    TestCommand c2(&s1);

    std::array<std::pair<Command *, bool>, 2> commands = {{{&c1, false}, {&c2, false}}};
    ASSERT_DEATH({ ConcurrentCommand<2> command(commands, "test command"); }, ".*");
}

TEST(ConcurrentCommands, command_has_deadline_command)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    TestCommand c1(&s1);
    TestSubsystem s2(&drivers);
    TestCommand deadlineCommand(&s2);

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);

    std::array<std::pair<Command *, bool>, 1> commands = {{{&c1, false}}};
    ConcurrentDeadlineCommand<1> command(commands, "concurrent deadline", &deadlineCommand);
    scheduler.addCommand(&command);

    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    deadlineCommand.setFinished(true);
    scheduler.run();
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

// repeat command tests
TEST(ConcurrentCommands, repeat_command_reruns_after_finishing)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    TestSubsystem s2(&drivers);
    scheduler.registerSubsystem(&s2);  // only once

    NiceMock<CommandMock> c1;
    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    TestCommand deadlineCommand(&s2);

    std::array<std::pair<Command *, bool>, 1> commands = {{{&c1, true}}};
    ConcurrentDeadlineCommand<1> command(commands, "repeat with deadline", &deadlineCommand);

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(2);
    scheduler.addCommand(&command);

    // Run 1: c1 finishes, enters waiting state
    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    // Run 2: c1 is ready, end(false) + re-initialize + execute
    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);
    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(false));
    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, repeat_command_waits_when_not_ready_to_reschedule)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    TestSubsystem s2(&drivers);
    scheduler.registerSubsystem(&s2);

    NiceMock<CommandMock> c1;  // repeat
    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    TestCommand deadlineCommand(&s2);

    std::array<std::pair<Command *, bool>, 1> commands = {{{&c1, true}}};
    ConcurrentDeadlineCommand<1> command(commands, "repeat not ready", &deadlineCommand);

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    scheduler.addCommand(&command);

    // c1 finishes
    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    scheduler.run();

    // c1 not ready, should not reinitialize or execute
    EXPECT_CALL(c1, isReady).WillOnce(Return(false));
    EXPECT_CALL(c1, initialize).Times(0);
    EXPECT_CALL(c1, execute).Times(0);
    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, repeat_command_ended_by_deadline_unconditionally)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    TestSubsystem s2(&drivers);
    scheduler.registerSubsystem(&s2);

    NiceMock<CommandMock> c1;  // repeat
    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    TestCommand deadlineCommand(&s2);

    std::array<std::pair<Command *, bool>, 1> commands = {{{&c1, true}}};
    ConcurrentDeadlineCommand<1> command(commands, "repeat deadline end", &deadlineCommand);

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    scheduler.addCommand(&command);

    // c1 finishes, deadline not yet done
    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    // deadline fires while c1 is in ended state
    deadlineCommand.setFinished(true);
    EXPECT_CALL(c1, isReady).WillOnce(Return(false));
    EXPECT_CALL(c1, end(false)).Times(1);  // ended by deadline command
    scheduler.run();
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}