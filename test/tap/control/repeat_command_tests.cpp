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

#include "tap/control/finite_repeat_command.hpp"
#include "tap/control/instant_command.hpp"
#include "tap/control/repeat_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/command_mock.hpp"

#include "test_command.hpp"
#include "test_subsystem.hpp"

using namespace tap::control;
using std::set;
using tap::Drivers;
using tap::mock::CommandMock;
using namespace testing;

TEST(RepeatCommand, instant_command_repeats)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem ts(&drivers);
    scheduler.registerSubsystem(&ts);

    bool toggle = false;
    std::function<void()> testAction = [&toggle]() { toggle = !toggle; };
    InstantCommand<1> ic(testAction, {&ts});
    RepeatCommand rc(&ic);
    scheduler.addCommand(&rc);

    scheduler.run();
    EXPECT_TRUE(toggle);
    scheduler.run();
    EXPECT_FALSE(toggle);
    scheduler.run();
    EXPECT_TRUE(toggle);
}

TEST(RepeatCommand, noninstant_command_repeats)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem ts(&drivers);
    scheduler.registerSubsystem(&ts);

    TestCommand tc(&ts);
    RepeatCommand rc(&tc);
    scheduler.addCommand(&rc);

    scheduler.run();
    EXPECT_FALSE(tc.isFinished());
    tc.setFinished(true);
    scheduler.run();
    EXPECT_TRUE(tc.isFinished());
}

Test(FiniteRepeatCommand, finite_repeat_command)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem ts(&drivers);
    scheduler.registerSubsystem(&ts);

    TestCommand tc(&ts);
    FiniteRepeatCommand rc(&tc, 2);

    scheduler.addCommand(&rc);
    EXPECT_EQ(rc.isReady());

    rc.execute();
    EXPECT_EQ(rc.getCurrentRepeatCount(), 1);
    rc.execute();
    EXPECT_EQ(rc.getCurrentRepeatCount(), 0);
    EXPECT_TRUE(rc.isFinished());
}