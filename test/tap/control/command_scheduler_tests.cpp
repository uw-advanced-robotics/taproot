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

#include <memory>

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/command_mock.hpp"
#include "tap/mock/remote_mock.hpp"
#include "tap/mock/subsystem_mock.hpp"

using std::set;
using tap::Drivers;
using tap::mock::CommandMock;
using tap::mock::RemoteMock;
using tap::mock::SubsystemMock;
using namespace tap::control;
using namespace testing;
using namespace std;

constexpr int MAX_CMDS_OR_SUBS =
    std::min(sizeof(command_scheduler_bitmap_t), sizeof(subsystem_scheduler_bitmap_t)) * 8;

class CommandSchedulerTest : public Test
{
protected:
    CommandSchedulerTest() : scheduler(&drivers, true) {}

    std::vector<unique_ptr<NiceMock<SubsystemMock>>> constructNSubsystemMocks(size_t n)
    {
        std::vector<unique_ptr<NiceMock<SubsystemMock>>> subs;
        for (size_t i = 0; i < n; i++)
        {
            auto sub = make_unique<NiceMock<SubsystemMock>>(&drivers);
            subs.push_back(move(sub));
        }
        return subs;
    }

    set<Subsystem *> subsToSet(const vector<unique_ptr<NiceMock<SubsystemMock>>> &subs)
    {
        std::set<Subsystem *> req;

        for (auto &sub : subs)
        {
            req.insert(sub.get());
        }

        return req;
    }

    Drivers drivers;
    CommandScheduler scheduler;
};

TEST_F(CommandSchedulerTest, constructor_multiple_master_schedulers_throws_error)
{
    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    CommandScheduler scheduler1(&drivers, true);
    CommandScheduler scheduler2(&drivers, true);
}

TEST_F(CommandSchedulerTest, constructor_single_master_schedulers_does_not_throw_error)
{
    CommandScheduler scheduler1(&drivers);
    CommandScheduler scheduler2(&drivers);
}

TEST_F(CommandSchedulerTest, registerSubsystem_single_subsystem_added_and_ran)
{
    SubsystemMock sub(&drivers);

    EXPECT_CALL(sub, refresh);
    EXPECT_CALL(sub, getDefaultCommand).WillOnce(Return(nullptr));

    scheduler.registerSubsystem(&sub);
    scheduler.run();
}

TEST_F(CommandSchedulerTest, registerSubsystem_multiple_unique_subsystems_added_and_ran)
{
    auto subMocks = constructNSubsystemMocks(3);

    for (auto &sub : subMocks)
    {
        EXPECT_CALL(*sub, refresh);
        EXPECT_CALL(*sub, getDefaultCommand).WillOnce(Return(nullptr));
        scheduler.registerSubsystem(sub.get());
    }

    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    registerSubsystem_single_subsystem_added_multiple_times_only_added_once)
{
    SubsystemMock sub(&drivers);

    EXPECT_CALL(sub, refresh);
    EXPECT_CALL(sub, getDefaultCommand).WillOnce(Return(nullptr));
    EXPECT_CALL(drivers.errorController, addToErrorList);

    scheduler.registerSubsystem(&sub);
    scheduler.registerSubsystem(&sub);
    scheduler.run();
}

TEST_F(CommandSchedulerTest, registerSubsystem_doesnt_register_nullptr_subsystem)
{
    EXPECT_CALL(drivers.errorController, addToErrorList);

    scheduler.registerSubsystem(nullptr);
    EXPECT_FALSE(scheduler.isSubsystemRegistered(nullptr));
}

TEST_F(CommandSchedulerTest, registerSubsystem_big_batch_subsystem_assertion_succeeds)
{
    auto subs = constructNSubsystemMocks(MAX_CMDS_OR_SUBS);

    for (int i = 0; i < MAX_CMDS_OR_SUBS / 2; i++)
    {
        scheduler.registerSubsystem(subs[i].get());
    }

    for (int i = 0; i < MAX_CMDS_OR_SUBS / 2; i++)
    {
        EXPECT_TRUE(scheduler.isSubsystemRegistered(subs[i].get()));
        EXPECT_FALSE(scheduler.isSubsystemRegistered(subs[i + MAX_CMDS_OR_SUBS / 2].get()));
    }
}

TEST_F(CommandSchedulerTest, isSubsystemRegistered_returns_true_if_registered)
{
    auto subs = constructNSubsystemMocks(3);

    for (auto &sub : subs)
    {
        scheduler.registerSubsystem(sub.get());
        EXPECT_TRUE(scheduler.isSubsystemRegistered(sub.get()));
    }
}

TEST_F(CommandSchedulerTest, isSubsystemRegistered_returns_false_if_not_registered)
{
    auto subs = constructNSubsystemMocks(3);

    scheduler.registerSubsystem(subs[0].get());
    EXPECT_FALSE(scheduler.isSubsystemRegistered(subs[1].get()));
    EXPECT_FALSE(scheduler.isSubsystemRegistered(subs[2].get()));
}

TEST_F(CommandSchedulerTest, addCommand_null_added_command_raises_error)
{
    // Expect an error when the command is null.
    EXPECT_CALL(drivers.errorController, addToErrorList);
    scheduler.addCommand(nullptr);
}

TEST_F(CommandSchedulerTest, addCommand_with_hardware_tests_running_raises_error)
{
    SubsystemMock sub(&drivers);
    NiceMock<CommandMock> cmd;

    EXPECT_CALL(drivers.errorController, addToErrorList);
    EXPECT_CALL(sub, setHardwareTestsIncomplete);

    scheduler.registerSubsystem(&sub);
    scheduler.startHardwareTests();
    scheduler.addCommand(&cmd);
}

TEST_F(CommandSchedulerTest, addCommand_with_no_subsystem_registered_raises_error)
{
    // Setup, create a subsystem but dont add it to command scheduler.
    SubsystemMock s(&drivers);
    std::set<Subsystem *> req{&s};
    NiceMock<CommandMock> c(req);

    // Expect an error with no subsystem in the command scheduler.
    EXPECT_CALL(drivers.errorController, addToErrorList);

    scheduler.addCommand(&c);
}

TEST_F(CommandSchedulerTest, addCommand_with_not_all_subsystems_registered_raises_error)
{
    auto subs = constructNSubsystemMocks(3);

    set<Subsystem *> req = subsToSet(subs);

    NiceMock<CommandMock> c(req);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(3);

    // No subsystems added, will fail.
    scheduler.addCommand(&c);

    // One subsystem registered, will fail.
    scheduler.registerSubsystem(subs[0].get());
    scheduler.addCommand(&c);

    // Two subsystems registered, will fail.
    scheduler.registerSubsystem(subs[1].get());
    scheduler.addCommand(&c);
}

TEST_F(CommandSchedulerTest, isCommandScheduled_nullptr_command_returns_false)
{
    EXPECT_FALSE(scheduler.isCommandScheduled(nullptr));
}

TEST_F(CommandSchedulerTest, isSubsystemRegistered_nullptr_subsystem_returns_false)
{
    EXPECT_FALSE(scheduler.isSubsystemRegistered(nullptr));
}

TEST_F(
    CommandSchedulerTest,
    addCommand_and_isCommandScheduled_command_successfully_added_with_single_dependent_subsystem)
{
    // Setup, create a subsystem but dont add it to command scheduler.
    SubsystemMock s(&drivers);
    set<Subsystem *> req{&s};
    NiceMock<CommandMock> c(req);

    // Set up another command that we can set an expectation on the `initialize` function.
    EXPECT_CALL(c, initialize);

    scheduler.registerSubsystem(&s);

    // Expect no errors, and the method initalize is called on the command.
    scheduler.addCommand(&c);
    EXPECT_TRUE(scheduler.isCommandScheduled(&c));
}

TEST_F(
    CommandSchedulerTest,
    addCommand_and_isCommandScheduled_command_added_successfully_with_multiple_subsystem_dependencies_all_registered)
{
    auto subs = constructNSubsystemMocks(3);

    set<Subsystem *> req = subsToSet(subs);
    NiceMock<CommandMock> c(req);

    EXPECT_CALL(c, initialize);

    // Add all subsystems, registering command will succeed
    for (auto &sub : subs)
    {
        scheduler.registerSubsystem(sub.get());
    }
    scheduler.addCommand(&c);
    EXPECT_TRUE(scheduler.isCommandScheduled(&c));
}

TEST_F(
    CommandSchedulerTest,
    addCommand_successfully_removes_previously_added_command_with_same_subsystem)
{
    NiceMock<SubsystemMock> s(&drivers);
    set<Subsystem *> req{&s};
    NiceMock<CommandMock> c1(req);
    NiceMock<CommandMock> c2(req);

    EXPECT_CALL(c2, execute);

    // Set up the test, adding the first command that will later be removed.
    scheduler.registerSubsystem(&s);
    scheduler.addCommand(&c1);
    scheduler.addCommand(&c2);

    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    addCommand_successfully_removes_previously_added_command_with_subset_of_subsystem_requirements)
{
    auto subs = constructNSubsystemMocks(3);

    set<Subsystem *> c1Req{subs[0].get(), subs[1].get()};
    set<Subsystem *> c2Req = subsToSet(subs);
    NiceMock<CommandMock> c1(c1Req);
    NiceMock<CommandMock> c2(c2Req);

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c2, execute);

    for (auto &sub : subs)
    {
        EXPECT_CALL(*sub, refresh);
        scheduler.registerSubsystem(sub.get());
    }

    // The first command is associated with the first two subsystems.
    scheduler.addCommand(&c1);
    // The second command is associated with all three subsystems.
    scheduler.addCommand(&c2);

    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    addCommand_multiple_successfully_removes_previously_added_command_if_single_shared_subsystem_requirement)
{
    auto subs = constructNSubsystemMocks(3);

    set<Subsystem *> c1Req{subs[0].get(), subs[1].get()};
    set<Subsystem *> c2Req{subs[1].get(), subs[2].get()};
    NiceMock<CommandMock> c1(c1Req);
    NiceMock<CommandMock> c2(c2Req);

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);

    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c2, execute);

    EXPECT_CALL(*subs[0], getDefaultCommand);

    for (auto &sub : subs)
    {
        scheduler.registerSubsystem(sub.get());
    }

    // The first command is associated with the first two subsystems.
    scheduler.addCommand(&c1);
    // The second command is associated with last two subsystems.
    scheduler.addCommand(&c2);

    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    addCommand_add_two_commands_with_overlapping_sub_requirements_but_second_addCommand_fails_so_first_still_remains_added)
{
    auto subs = constructNSubsystemMocks(3);

    set<Subsystem *> c1Req{subs[0].get(), subs[1].get()};
    set<Subsystem *> c2Req{subs[1].get(), subs[2].get()};
    set<Subsystem *> c3Req{subs[0].get()};

    NiceMock<CommandMock> c1(c1Req);
    NiceMock<CommandMock> c2(c2Req);
    NiceMock<CommandMock> c3(c3Req);

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c3, initialize);
    EXPECT_CALL(c3, execute);
    EXPECT_CALL(drivers.errorController, addToErrorList);
    EXPECT_CALL(*subs[1], getDefaultCommand);

    scheduler.registerSubsystem(subs[0].get());
    scheduler.registerSubsystem(subs[1].get());

    // The first command is associated with the first two subsystems.
    scheduler.addCommand(&c1);
    // The second command is associated with last two subsystems. This should fail.
    scheduler.addCommand(&c2);
    EXPECT_FALSE(scheduler.isCommandScheduled(&c2));
    scheduler.addCommand(&c3);
    EXPECT_FALSE(scheduler.isCommandScheduled(&c2));

    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    addCommand_multiple_successfully_removes_previously_added_command_with_superset_of_subsystems)
{
    auto subs = constructNSubsystemMocks(3);

    set<Subsystem *> c1Req = subsToSet(subs);
    set<Subsystem *> c2Req{subs[0].get(), subs[1].get()};

    NiceMock<CommandMock> c1(c1Req);
    NiceMock<CommandMock> c2(c2Req);

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);

    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c2, execute);

    // The third subsystem doesn't have a command associated with it.
    EXPECT_CALL(*subs[2], getDefaultCommand).WillOnce(Return(nullptr));

    for (auto &sub : subs)
    {
        scheduler.registerSubsystem(sub.get());
    }

    // The first command is associated with all three subsystems.
    scheduler.addCommand(&c1);
    // When the second command is added, the first command is removed completely from the
    scheduler.addCommand(&c2);

    scheduler.run();
}

TEST_F(CommandSchedulerTest, addCommand_add_multiple_commands_with_overlapping_sub_requirements)
{
    auto subs = constructNSubsystemMocks(5);

    set<Subsystem *> c1Req{subs[0].get(), subs[1].get(), subs[2].get()};
    set<Subsystem *> c2Req{subs[0].get(), subs[1].get()};
    set<Subsystem *> c3Req{subs[2].get(), subs[3].get()};
    set<Subsystem *> c4Req{subs[1].get(), subs[2].get(), subs[4].get()};

    NiceMock<CommandMock> c1(c1Req);
    NiceMock<CommandMock> c2(c2Req);
    NiceMock<CommandMock> c3(c3Req);
    NiceMock<CommandMock> c4(c4Req);

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c2, end);
    EXPECT_CALL(c3, initialize);
    EXPECT_CALL(c3, end);
    EXPECT_CALL(c4, initialize);
    EXPECT_CALL(*subs[0], getDefaultCommand);
    EXPECT_CALL(*subs[3], getDefaultCommand);
    EXPECT_CALL(c4, execute);

    for (auto &sub : subs)
    {
        scheduler.registerSubsystem(sub.get());
    }

    scheduler.addCommand(&c1);
    scheduler.addCommand(&c2);
    scheduler.addCommand(&c3);

    EXPECT_TRUE(scheduler.isCommandScheduled(&c2));
    EXPECT_TRUE(scheduler.isCommandScheduled(&c3));

    scheduler.addCommand(&c4);

    scheduler.run();
}

TEST_F(CommandSchedulerTest, startHardwareTests_doesnot_segfault_with_nullptr_commands)
{
    auto subs = constructNSubsystemMocks(2);

    for (auto &sub : subs)
    {
        EXPECT_CALL(*sub, setHardwareTestsIncomplete);
        scheduler.registerSubsystem(sub.get());
    }

    scheduler.startHardwareTests();
}

TEST_F(CommandSchedulerTest, startHardwareTests_removes_all_commands_in_scheduler)
{
    auto subs = constructNSubsystemMocks(2);
    set<Subsystem *> c1Req{subs[0].get()};
    set<Subsystem *> c2Req{subs[1].get()};
    NiceMock<CommandMock> c1(c1Req);
    NiceMock<CommandMock> c2(c2Req);

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, end);

    for (auto &sub : subs)
    {
        EXPECT_CALL(*sub, setHardwareTestsIncomplete);
        scheduler.registerSubsystem(sub.get());
    }

    scheduler.addCommand(&c1);
    scheduler.addCommand(&c2);
    scheduler.startHardwareTests();

    EXPECT_FALSE(scheduler.isCommandScheduled(&c1));
    EXPECT_FALSE(scheduler.isCommandScheduled(&c2));
    EXPECT_TRUE(scheduler.isSubsystemRegistered(subs[0].get()));
    EXPECT_TRUE(scheduler.isSubsystemRegistered(subs[1].get()));
}

TEST_F(
    CommandSchedulerTest,
    startHardwareTests_single_command_registered_to_multiple_subsystems_only_ended_once)
{
    auto subs = constructNSubsystemMocks(2);
    set<Subsystem *> c1Req{subs[0].get(), subs[1].get()};
    NiceMock<CommandMock> c1(c1Req);

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);

    for (auto &sub : subs)
    {
        EXPECT_CALL(*sub, setHardwareTestsIncomplete);
        scheduler.registerSubsystem(sub.get());
    }

    scheduler.addCommand(&c1);
    scheduler.startHardwareTests();

    EXPECT_FALSE(scheduler.isCommandScheduled(&c1));
    EXPECT_TRUE(scheduler.isSubsystemRegistered(subs[0].get()));
    EXPECT_TRUE(scheduler.isSubsystemRegistered(subs[1].get()));
}

TEST_F(
    CommandSchedulerTest,
    stopHardwareTests_calls_setHardwareTestsComplete_on_all_subsystems_and_allows_adding_commands)
{
    auto subs = constructNSubsystemMocks(2);
    set<Subsystem *> c1Req{subs[0].get()};
    set<Subsystem *> c2Req{subs[1].get()};
    NiceMock<CommandMock> c1(c1Req);
    NiceMock<CommandMock> c2(c2Req);

    for (auto &sub : subs)
    {
        EXPECT_CALL(*sub, setHardwareTestsIncomplete);
        EXPECT_CALL(*sub, setHardwareTestsComplete);
        scheduler.registerSubsystem(sub.get());
    }

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c2, initialize);

    scheduler.startHardwareTests();
    scheduler.stopHardwareTests();
    scheduler.addCommand(&c1);
    scheduler.addCommand(&c2);

    EXPECT_TRUE(scheduler.isCommandScheduled(&c1));
    EXPECT_TRUE(scheduler.isCommandScheduled(&c2));
    EXPECT_TRUE(scheduler.isSubsystemRegistered(subs[0].get()));
    EXPECT_TRUE(scheduler.isSubsystemRegistered(subs[1].get()));
}

TEST_F(CommandSchedulerTest, run_with_hardware_tests_enabled_calls_runHardwareTests_and_refresh)
{
    auto subs = constructNSubsystemMocks(3);

    // Won't call runHardwareTests for this subsystem since the test is complete.
    ON_CALL(*subs[0], isHardwareTestComplete).WillByDefault(Return(true));

    EXPECT_CALL(*subs[0], setHardwareTestsIncomplete);
    EXPECT_CALL(*subs[0], runHardwareTests).Times(0);
    EXPECT_CALL(*subs[0], refresh);

    scheduler.registerSubsystem(subs[0].get());

    for (size_t i = 1; i < subs.size(); i++)
    {
        auto &sub = subs[i];

        ON_CALL(*sub, isHardwareTestComplete).WillByDefault(Return(false));

        EXPECT_CALL(*sub, setHardwareTestsIncomplete);
        EXPECT_CALL(*sub, runHardwareTests);
        EXPECT_CALL(*sub, refresh);

        scheduler.registerSubsystem(sub.get());
    }

    scheduler.startHardwareTests();
    scheduler.run();
}

TEST_F(CommandSchedulerTest, run_with_single_registered_subsystem_calls_refresh)
{
    SubsystemMock s1(&drivers);

    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s1, getDefaultCommand).WillOnce(Return(nullptr));

    scheduler.registerSubsystem(&s1);
    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    run_with_single_registered_subsystem_and_single_command_calls_refresh_and_execute)
{
    constexpr int RUN_TIMES = 100;

    NiceMock<SubsystemMock> s1(&drivers);
    set<Subsystem *> req{&s1};
    NiceMock<CommandMock> c1(req);

    EXPECT_CALL(s1, refresh).Times(RUN_TIMES);
    EXPECT_CALL(c1, execute).Times(RUN_TIMES);
    EXPECT_CALL(c1, isFinished).Times(RUN_TIMES).WillRepeatedly(Return(false));
    EXPECT_CALL(c1, initialize);

    scheduler.registerSubsystem(&s1);
    scheduler.addCommand(&c1);

    for (int i = 0; i < RUN_TIMES; i++)
    {
        scheduler.run();
    }
}

TEST_F(
    CommandSchedulerTest,
    run_with_many_registered_subsystems_and_commands_calls_refresh_and_execute)
{
    constexpr int RUN_TIMES = 100;

    auto subs = constructNSubsystemMocks(MAX_CMDS_OR_SUBS);
    vector<unique_ptr<NiceMock<CommandMock>>> cmds;

    for (size_t i = 0; i < subs.size(); i++)
    {
        set<Subsystem *> req{subs[i].get()};
        auto cmd = make_unique<NiceMock<CommandMock>>(req);
        cmds.push_back(move(cmd));

        EXPECT_CALL(*subs[i], refresh).Times(RUN_TIMES);

        EXPECT_CALL(*cmds[i], execute).Times(RUN_TIMES);
        EXPECT_CALL(*cmds[i], isFinished).Times(RUN_TIMES);
        EXPECT_CALL(*cmds[i], initialize);

        scheduler.registerSubsystem(subs[i].get());
        scheduler.addCommand(cmds[i].get());
    }

    uint32_t totalTime = 0;
    for (int i = 0; i < RUN_TIMES; i++)
    {
        uint32_t start = tap::arch::clock::getTimeMicroseconds();
        scheduler.run();
        uint32_t dt = tap::arch::clock::getTimeMicroseconds() - start;
        totalTime += dt;
    }

    std::cout << "             Average time to run scheduler with " << MAX_CMDS_OR_SUBS
              << " subsystems and commands, " << RUN_TIMES << " samples: " << totalTime / RUN_TIMES
              << " microseconds" << std::endl;
}

TEST_F(CommandSchedulerTest, addCommand_disjoint_required_subsystems_successfully_added)
{
    auto subs = constructNSubsystemMocks(6);

    set<Subsystem *> c1Req{subs[0].get(), subs[4].get()};
    set<Subsystem *> c2Req{subs[1].get(), subs[2].get()};
    set<Subsystem *> c3Req{subs[3].get(), subs[5].get()};

    NiceMock<CommandMock> c1(c1Req);
    NiceMock<CommandMock> c2(c2Req);
    NiceMock<CommandMock> c3(c3Req);

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c3, initialize);

    for (auto &sub : subs)
    {
        scheduler.registerSubsystem(sub.get());
    }

    scheduler.addCommand(&c1);
    scheduler.addCommand(&c2);
    scheduler.addCommand(&c3);

    EXPECT_TRUE(scheduler.isCommandScheduled(&c1));
    EXPECT_TRUE(scheduler.isCommandScheduled(&c2));
    EXPECT_TRUE(scheduler.isCommandScheduled(&c3));

    EXPECT_CALL(c1, execute);
    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c3, execute);
    EXPECT_CALL(c1, isFinished);
    EXPECT_CALL(c2, isFinished);
    EXPECT_CALL(c3, isFinished);

    scheduler.run();
}

TEST_F(CommandSchedulerTest, run_large_number_of_subsystem_requirements_successful)
{
    const int NUM_DEPENDENT_SUBS = 50;

    auto subs = constructNSubsystemMocks(NUM_DEPENDENT_SUBS);
    auto reqs = subsToSet(subs);
    NiceMock<CommandMock> c(reqs);

    EXPECT_CALL(c, initialize);
    EXPECT_CALL(c, execute);

    for (auto &sub : subs)
    {
        scheduler.registerSubsystem(sub.get());
    }

    scheduler.addCommand(&c);
    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    run_subsystem_with_default_command_requiring_single_subsystem_added_successfully)
{
    SubsystemMock s(&drivers);
    NiceMock<CommandMock> c;

    EXPECT_CALL(s, getDefaultCommand).WillOnce(Return(&c));
    EXPECT_CALL(s, refresh).Times(2);
    EXPECT_CALL(c, initialize);
    EXPECT_CALL(c, getRequirementsBitwise).WillOnce(Return(1UL << s.getGlobalIdentifier()));

    scheduler.registerSubsystem(&s);
    scheduler.run();

    EXPECT_CALL(c, execute);
    EXPECT_CALL(c, isFinished);

    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    run_subsystem_with_default_command_requiring_multiple_subsystems_added_currectly)
{
    const int NUM_DEPENDENT_SUBS = 50;

    auto subs = constructNSubsystemMocks(NUM_DEPENDENT_SUBS);
    auto reqs = subsToSet(subs);
    NiceMock<CommandMock> c(reqs);

    EXPECT_CALL(*subs[0], getDefaultCommand).WillOnce(Return(&c));

    for (auto &sub : subs)
    {
        scheduler.registerSubsystem(sub.get());
    }

    EXPECT_CALL(c, initialize);

    scheduler.run();

    EXPECT_CALL(c, execute);
    EXPECT_CALL(c, isFinished);

    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    run_subsystem_with_default_command_that_depends_on_other_subsystem_that_already_has_command_is_added_successfully)
{
    auto subs = constructNSubsystemMocks(2);

    set<Subsystem *> c1Req{subs[0].get()};
    set<Subsystem *> c2Req{subs[0].get(), subs[1].get()};

    NiceMock<CommandMock> c1(c1Req);
    NiceMock<CommandMock> c2(c2Req);

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, execute);
    EXPECT_CALL(c1, isFinished);
    EXPECT_CALL(c1, end);

    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c2, isFinished);

    EXPECT_CALL(*subs[1], getDefaultCommand).WillOnce(Return(&c2));

    for (auto &sub : subs)
    {
        scheduler.registerSubsystem(sub.get());
    }

    scheduler.addCommand(&c1);

    scheduler.run();
    scheduler.run();
}

TEST_F(
    CommandSchedulerTest,
    run_command_ending_removed_from_scheduler_and_default_command_for_subsystem_automatically_added)
{
    SubsystemMock s(&drivers);
    set<Subsystem *> req{&s};
    NiceMock<CommandMock> c1(req);
    NiceMock<CommandMock> c2(req);

    EXPECT_CALL(s, refresh).Times(2);
    EXPECT_CALL(s, getDefaultCommand).WillOnce(Return(&c2));

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, execute);
    EXPECT_CALL(c1, end);
    ON_CALL(c1, isFinished).WillByDefault(Return(true));

    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c2, initialize);

    scheduler.registerSubsystem(&s);
    scheduler.addCommand(&c1);

    scheduler.run();
    scheduler.run();
}

TEST_F(CommandSchedulerTest, run_default_command_that_naturally_ends_always_rescheduled)
{
    NiceMock<SubsystemMock> s(&drivers);
    set<Subsystem *> req{&s};
    NiceMock<CommandMock> c(req);

    EXPECT_CALL(c, initialize).Times(3);
    EXPECT_CALL(c, execute).Times(2);
    EXPECT_CALL(c, isFinished).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(c, end).Times(2);

    ON_CALL(s, getDefaultCommand).WillByDefault(Return(&c));

    scheduler.registerSubsystem(&s);
    scheduler.run();
    scheduler.run();
    scheduler.run();
}

TEST_F(CommandSchedulerTest, run_in_safe_disconnect_mode_calls_inertRefresh)
{
    NiceMock<SubsystemMock> s(&drivers);

    EXPECT_CALL(s, inertRefresh);
    EXPECT_CALL(s, refresh).Times(0);

    scheduler.setSetSchedulerInertFn([]() { return true; });
    scheduler.registerSubsystem(&s);

    scheduler.run();
}

TEST_F(CommandSchedulerTest, run_in_normal_mode_calls_refresh_inert_fn_false)
{
    NiceMock<SubsystemMock> s(&drivers);

    EXPECT_CALL(s, inertRefresh).Times(0);
    EXPECT_CALL(s, refresh);

    scheduler.setSetSchedulerInertFn([]() { return false; });
    scheduler.registerSubsystem(&s);

    scheduler.run();
}

TEST_F(CommandSchedulerTest, removeCommand_nullptr_command_doesnt_crash)
{
    EXPECT_CALL(drivers.errorController, addToErrorList);
    scheduler.removeCommand(nullptr, false);
}

TEST_F(CommandSchedulerTest, removeCommand_single_cmd_in_scheduler_removed_and_end_called)
{
    SubsystemMock sub(&drivers);
    set<Subsystem *> req{&sub};
    NiceMock<CommandMock> cmd(req);

    EXPECT_CALL(cmd, initialize).Times(2);
    EXPECT_CALL(cmd, end(true));
    EXPECT_CALL(cmd, end(false));

    scheduler.registerSubsystem(&sub);

    scheduler.addCommand(&cmd);
    scheduler.removeCommand(&cmd, true);

    scheduler.addCommand(&cmd);
    scheduler.removeCommand(&cmd, false);
}

TEST_F(CommandSchedulerTest, removeCommand_single_cmd_removed_from_big_batch)
{
    auto subs = constructNSubsystemMocks(MAX_CMDS_OR_SUBS);
    vector<unique_ptr<NiceMock<CommandMock>>> cmds;

    for (uint32_t i = 0; i < subs.size(); i++)
    {
        set<Subsystem *> req{subs[i].get()};

        auto cmd = make_unique<NiceMock<CommandMock>>(req);
        cmds.push_back(move(cmd));

        scheduler.registerSubsystem(subs[i].get());
        scheduler.addCommand(cmds[i].get());
    }

    EXPECT_CALL(*cmds[0], end(true));
    EXPECT_CALL(*cmds[1], end(false));

    scheduler.removeCommand(cmds[0].get(), true);
    scheduler.removeCommand(cmds[1].get(), false);

    for (uint32_t i = 2; i < cmds.size(); i++)
    {
        EXPECT_TRUE(scheduler.isCommandScheduled(cmds[i].get()));
    }
}

TEST_F(
    CommandSchedulerTest,
    subsystemListSize_commandListSize_returns_number_of_subs_cmds_in_scheduler)
{
    auto subs = constructNSubsystemMocks(MAX_CMDS_OR_SUBS);
    vector<unique_ptr<NiceMock<CommandMock>>> cmds;

    for (int i = 0; i < MAX_CMDS_OR_SUBS; i++)
    {
        EXPECT_EQ(i, scheduler.subsystemListSize());
        EXPECT_EQ(i, scheduler.commandListSize());

        set<Subsystem *> req{subs[i].get()};
        auto cmd = make_unique<NiceMock<CommandMock>>(req);
        cmds.push_back(move(cmd));

        EXPECT_CALL(*cmds[i], initialize);
        scheduler.registerSubsystem(subs[i].get());
        scheduler.addCommand(cmds[i].get());
    }

    // Remove some commands from middle to test size calculation
    EXPECT_CALL(*cmds[0], end);
    EXPECT_CALL(*cmds[10], end);
    EXPECT_CALL(*cmds[20], end);

    scheduler.removeCommand(cmds[0].get(), true);
    EXPECT_EQ(MAX_CMDS_OR_SUBS - 1, scheduler.commandListSize());
    scheduler.removeCommand(cmds[10].get(), true);
    EXPECT_EQ(MAX_CMDS_OR_SUBS - 2, scheduler.commandListSize());
    scheduler.removeCommand(cmds[20].get(), true);
    EXPECT_EQ(MAX_CMDS_OR_SUBS - 3, scheduler.commandListSize());
}

TEST_F(CommandSchedulerTest, iterators_nothing_in_scheduler_returns_null)
{
    auto cmdIt = scheduler.cmdMapBegin();
    auto subIt = scheduler.subMapBegin();

    EXPECT_EQ(nullptr, *cmdIt);
    EXPECT_EQ(nullptr, *subIt);
    EXPECT_EQ(scheduler.cmdMapEnd(), cmdIt);
    EXPECT_EQ(scheduler.subMapEnd(), subIt);
}

TEST_F(CommandSchedulerTest, iterators_couple_cmd_sub_iterated_through)
{
    // Add a couple subs and cmds to the scheduler
    SubsystemMock sub1(&drivers);
    SubsystemMock sub2(&drivers);

    set<Subsystem *> c1Req{&sub1};
    set<Subsystem *> c2Req{&sub2};
    NiceMock<CommandMock> cmd1(c1Req);
    NiceMock<CommandMock> cmd2(c2Req);

    EXPECT_CALL(cmd1, initialize);
    EXPECT_CALL(cmd2, initialize);

    scheduler.registerSubsystem(&sub1);
    scheduler.registerSubsystem(&sub2);
    scheduler.addCommand(&cmd1);
    scheduler.addCommand(&cmd2);

    auto cmdIt = scheduler.cmdMapBegin();
    EXPECT_EQ(&cmd1, *cmdIt);
    EXPECT_EQ(&cmd2, *(++cmdIt));
    cmdIt++;
    EXPECT_EQ(nullptr, *cmdIt);

    auto subIt = scheduler.subMapBegin();
    EXPECT_EQ(&sub1, *subIt);
    EXPECT_EQ(&sub2, *(++subIt));
    subIt++;
    EXPECT_EQ(nullptr, *subIt);
}

TEST_F(CommandSchedulerTest, iterators_many_cmds_subs_iterated_through_using_foreach)
{
    auto subs = constructNSubsystemMocks(MAX_CMDS_OR_SUBS);
    vector<unique_ptr<NiceMock<CommandMock>>> cmds;

    for (uint32_t i = 0; i < MAX_CMDS_OR_SUBS; i++)
    {
        set<Subsystem *> req{subs[i].get()};
        auto cmd = make_unique<NiceMock<CommandMock>>(req);
        cmds.push_back(move(cmd));

        EXPECT_CALL(*cmds[i], initialize);
        scheduler.registerSubsystem(subs[i].get());
        scheduler.addCommand(cmds[i].get());
    }

    int i = 0;
    // Foreach with subsystem map
    std::for_each(scheduler.subMapBegin(), scheduler.subMapEnd(), [&](Subsystem *sub) {
        EXPECT_EQ(subs[i].get(), sub);
        i++;
    });

    i = 0;
    // Foreach with command map
    std::for_each(scheduler.cmdMapBegin(), scheduler.cmdMapEnd(), [&](Command *cmd) {
        EXPECT_EQ(cmds[i].get(), cmd);
        i++;
    });
}

TEST_F(CommandSchedulerTest, iterators_work_properly_with_gaps_in_global_registrar)
{
    auto subs = constructNSubsystemMocks(MAX_CMDS_OR_SUBS);
    vector<unique_ptr<NiceMock<CommandMock>>> cmds;

    for (size_t i = 0; i < subs.size(); i++)
    {
        if (i % 2 == 0)
        {
            scheduler.registerSubsystem(subs[i].get());
            auto cmd = make_unique<NiceMock<CommandMock>>();
            cmds.push_back(move(cmd));
        }
        else
        {
            set<Subsystem *> req{subs[i - 1].get()};
            auto cmd = make_unique<NiceMock<CommandMock>>(req);
            cmds.push_back(move(cmd));

            EXPECT_CALL(*cmds[i], initialize);
            scheduler.addCommand(cmds[i].get());
        }
    }

    int i = 0;
    for (auto it = scheduler.subMapBegin(); it != scheduler.subMapEnd(); it++)
    {
        EXPECT_EQ(subs[2 * i].get(), *it);
        i++;
    }

    i = 0;
    for (auto it = scheduler.cmdMapBegin(); it != scheduler.cmdMapEnd(); it++)
    {
        EXPECT_EQ(cmds[2 * i + 1].get(), *it);
        i++;
    }
}

TEST_F(CommandSchedulerTest, subsystem_iterator_iterator_begin_with_invalid_index)
{
    SubsystemMock sub1(&drivers);
    SubsystemMock sub2(&drivers);

    // Only add the second subsystem, so that the subsystem iterator starts on a subsystem that
    // is not in the scheduler
    scheduler.registerSubsystem(&sub2);

    auto it = scheduler.subMapBegin();
    EXPECT_EQ(&sub2, *it);
    EXPECT_EQ(scheduler.subMapEnd(), ++it);
}

TEST_F(CommandSchedulerTest, addCommand_doesnt_add_if_command_not_ready)
{
    NiceMock<CommandMock> c1;

    EXPECT_CALL(c1, isReady).WillOnce(Return(false));

    scheduler.addCommand(&c1);
    bool isScheduled = scheduler.isCommandScheduled(&c1);

    EXPECT_EQ(isScheduled, false);
}
