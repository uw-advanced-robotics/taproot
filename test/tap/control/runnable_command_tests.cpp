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

#include "tap/control/runnable_command.hpp"
#include "tap/drivers.hpp"

#include "test_subsystem.hpp"

using namespace tap::control;
using tap::Drivers;

TEST(RunnableCommand, runs_action_every_iteration_until_interrupted)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem subsystem(&drivers);
    scheduler.registerSubsystem(&subsystem);

    int runs = 0;
    RunnableCommand<1> command([&runs]() { ++runs; }, {&subsystem});

    EXPECT_FALSE(command.isFinished());
    EXPECT_EQ(
        command.getRequirementsBitwise(),
        static_cast<subsystem_scheduler_bitmap_t>(1) << subsystem.getGlobalIdentifier());

    scheduler.addCommand(&command);
    EXPECT_EQ(runs, 0);

    scheduler.run();
    scheduler.run();
    EXPECT_EQ(runs, 2);
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    scheduler.removeCommand(&command, true);
    scheduler.run();
    EXPECT_EQ(runs, 2);
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}
