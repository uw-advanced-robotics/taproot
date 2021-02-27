/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/command.hpp>
#include <aruwlib/control/command_scheduler.hpp>
#include <aruwsrc/control/example/example_command.hpp>
#include <aruwsrc/control/example/example_subsystem.hpp>
#include <gtest/gtest.h>

#include "aruwlib/mock/CommandMock.hpp"

using aruwlib::mock::CommandMock;

TEST(CommandScheduler, NullAddedCommandRaisesError)
{
    // Setup
    aruwlib::Drivers drivers;
    aruwlib::control::CommandScheduler instance(&drivers);

    // Expect an error when the command is null
    EXPECT_CALL(drivers.errorController, addToErrorList);
    instance.addCommand(nullptr);
}

TEST(CommandScheduler, AddedCommandWithNoSubsystemRaisesError)
{
    // Setup, create a subsystem but dont add it to command scheduler
    aruwlib::Drivers drivers;
    EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler).Times(2);
    EXPECT_CALL(drivers.djiMotorTxHandler, removeFromMotorManager).Times(2);
    aruwlib::control::CommandScheduler instance(&drivers);
    aruwsrc::control::ExampleSubsystem s(&drivers);
    aruwsrc::control::ExampleCommand c(&s, 100);

    // Expect an error with no subsystem in the command scheduler
    EXPECT_CALL(drivers.errorController, addToErrorList);
    instance.addCommand(&c);
}

TEST(CommandScheduler, CommandIsAddedProperly)
{
    // Setup, create a subsystem but dont add it to command scheduler
    aruwlib::Drivers drivers;
    EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler).Times(2);
    EXPECT_CALL(drivers.djiMotorTxHandler, removeFromMotorManager).Times(2);
    aruwlib::control::CommandScheduler instance(&drivers);
    aruwsrc::control::ExampleSubsystem s(&drivers);
    CommandMock c;

    // Set up another command that we can set an expectation on the `initialize` function.
    std::set<aruwlib::control::Subsystem*> cmdMockRequirements{&s};
    EXPECT_CALL(c, getRequirements)
        .Times(1)
        .WillRepeatedly(testing::ReturnRef(cmdMockRequirements));
    EXPECT_CALL(c, initialize);

    instance.registerSubsystem(&s);

    // Expect no errors, and the method initalize is called on the command
    instance.addCommand(&c);
}
