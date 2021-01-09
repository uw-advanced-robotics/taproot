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

#ifndef COMMAND_SCHEDULER_MOCK_HPP_
#define COMMAND_SCHEDULER_MOCK_HPP_

#include <aruwlib/control/command.hpp>
#include <aruwlib/control/command_scheduler.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class CommandSchedulerMock : public aruwlib::control::CommandScheduler
{
public:
    CommandSchedulerMock(aruwlib::Drivers *drivers) : aruwlib::control::CommandScheduler(drivers) {}
    MOCK_METHOD(void, run, (), (override));
    MOCK_METHOD(
        void,
        removeCommand,
        (aruwlib::control::Command * command, bool interrupted),
        (override));
    MOCK_METHOD(void, registerSubsystem, (aruwlib::control::Subsystem * subsystem), (override));
    MOCK_METHOD(
        bool,
        isSubsystemRegistered,
        (aruwlib::control::Subsystem * subsystem),
        (const override));
    MOCK_METHOD(bool, isCommandScheduled, (aruwlib::control::Command * command), (const override));
    MOCK_METHOD(void, addCommand, (aruwlib::control::Command * commandToAdd), (override));
};  // class CommandSchedulerMock
}  // namespace mock
}  // namespace aruwlib

#endif  // COMMAND_SCHEDULER_MOCK_HPP_
