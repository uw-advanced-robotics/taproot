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

#include "tap/control/instant_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/command_mock.hpp"

#include "test_subsystem.hpp"

using namespace tap::control;
using std::set;
using tap::Drivers;
using tap::mock::CommandMock;
using namespace testing;

TEST(InstantCommand, command_runs_instantly)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    TestSubsystem ts(&drivers);
    scheduler.registerSubsystem(&ts);

    bool ran = false;
    std::function<void()> testAction = [&ran]() { ran = true; };
    InstantCommand<1> ic(&scheduler, testAction, {&ts});

    EXPECT_FALSE(ran);
    scheduler.addCommand(&ic);

    scheduler.run();
    EXPECT_TRUE(ran);
    EXPECT_FALSE(scheduler.isCommandScheduled(&ic));
}