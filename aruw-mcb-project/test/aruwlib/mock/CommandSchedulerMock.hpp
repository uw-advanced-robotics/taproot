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
class CommandSchedulerMock : public control::CommandScheduler
{
public:
    CommandSchedulerMock(Drivers *drivers) : control::CommandScheduler(drivers) {}
    MOCK_METHOD(void, run, (), (override));
    MOCK_METHOD(void, addCommand, (control::Command *), (override));
    MOCK_METHOD(void, removeCommand, (control::Command *, bool), (override));
    MOCK_METHOD(bool, isCommandScheduled, (control::Command *), (const override));
    MOCK_METHOD(void, registerSubsystem, (control::Subsystem *), (override));
    MOCK_METHOD(bool, isSubsystemRegistered, (control::Subsystem *), (const override));
    MOCK_METHOD(void, startHardwareTests, (), (override));
    MOCK_METHOD(void, stopHardwareTests, (), (override));
    MOCK_METHOD(int, subsystemListSize, (), (const override));
    MOCK_METHOD(int, commandListSize, (), (const override));
    MOCK_METHOD(CommandIterator, cmdMapBegin, (), (override));
    MOCK_METHOD(CommandIterator, cmdMapEnd, (), (override));
    MOCK_METHOD(SubsystemIterator, subMapBegin, (), (override));
    MOCK_METHOD(SubsystemIterator, subMapEnd, (), (override));
    MOCK_METHOD(
        control::subsystem_scheduler_bitmap_t,
        getRegisteredSubsystemBitmap,
        (),
        (const override));
    MOCK_METHOD(control::command_scheduler_bitmap_t, getAddedCommandBitmap, (), (const override));
};  // class CommandSchedulerMock
}  // namespace mock
}  // namespace aruwlib

#endif  // COMMAND_SCHEDULER_MOCK_HPP_
