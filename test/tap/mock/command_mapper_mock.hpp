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

#ifndef TAPROOT_COMMAND_MAPPER_MOCK_HPP_
#define TAPROOT_COMMAND_MAPPER_MOCK_HPP_

#include <gmock/gmock.h>
#include <memory>

#include "tap/control/command.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/trigger_binding.hpp"

namespace tap
{
namespace mock
{
class CommandMapperMock : public tap::control::CommandMapper
{
public:
    CommandMapperMock(tap::Drivers *drivers);
    virtual ~CommandMapperMock();

    MOCK_METHOD(
        void,
        pollTriggerBindings,
        (), (override));
    MOCK_METHOD(void, addTriggerBinding, (std::unique_ptr<tap::control::TriggerBinding>), (override));
    MOCK_METHOD(std::size_t, getSize, (), (const override));
};  // class CommandMapperMock
}  // namespace mock
}  // namespace tap

#endif  // TAPROOT_COMMAND_MAPPER_MOCK_HPP_
