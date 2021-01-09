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

#ifndef COMMAND_MAPPER_MOCK_HPP_
#define COMMAND_MAPPER_MOCK_HPP_

#include <aruwlib/control/CommandMapper.hpp>
#include <aruwlib/control/command.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class CommandMapperMock : public aruwlib::control::CommandMapper
{
public:
    CommandMapperMock(aruwlib::Drivers *drivers) : aruwlib::control::CommandMapper(drivers) {}

    MOCK_METHOD(
        void,
        handleKeyStateChange,
        (uint16_t key,
         Remote::SwitchState leftSwitch,
         Remote::SwitchState rightSwitch,
         bool mouseL,
         bool mouseR),
        (override));
    MOCK_METHOD(
        void,
        addHoldMapping,
        (const aruwlib::control::RemoteMapState &mapping,
         const std::vector<aruwlib::control::Command *> commands),
        (override));
    MOCK_METHOD(
        void,
        addHoldRepeatMapping,
        (const aruwlib::control::RemoteMapState &mapping,
         const std::vector<aruwlib::control::Command *> commands),
        (override));
    MOCK_METHOD(
        void,
        addToggleMapping,
        (const aruwlib::control::RemoteMapState &mapping,
         const std::vector<aruwlib::control::Command *> commands),
        (override));
    MOCK_METHOD(
        void,
        addPressMapping,
        (const aruwlib::control::RemoteMapState &mapping,
         const std::vector<aruwlib::control::Command *> commands),
        (override));
    MOCK_METHOD(std::size_t, getSize, (), (const override));
};  // class CommandMapperMock
}  // namespace mock
}  // namespace aruwlib

#endif  // COMMAND_MAPPER_MOCK_HPP_
