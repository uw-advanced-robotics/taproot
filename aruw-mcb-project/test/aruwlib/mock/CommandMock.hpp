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

#ifndef COMMAND_MOCK_HPP_
#define COMMAND_MOCK_HPP_

#include <aruwlib/control/command.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class CommandMock : public control::Command
{
public:
    CommandMock() : Command()
    {
        // Most of the time tests expect that we are adding commands that
        // are ready to be added. This makes tests cleaner
        ON_CALL(*this, isReady).WillByDefault(testing::Return(true));
    }
    virtual ~CommandMock() = default;
    MOCK_METHOD(
        control::subsystem_scheduler_bitmap_t,
        getRequirementsBitwise,
        (),
        (const override));
    MOCK_METHOD(void, addSubsystemRequirement, (control::Subsystem * requirement), (override));
    MOCK_METHOD(const char*, getName, (), (const override));
    MOCK_METHOD(bool, isReady, (), (override));
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, execute, (), (override));
    MOCK_METHOD(void, end, (bool interrupted), (override));
    MOCK_METHOD(bool, isFinished, (), (const override));
};  // class CommandMock
}  // namespace mock
}  // namespace aruwlib

#endif  // COMMAND_MOCK_HPP_
