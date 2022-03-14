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

#include "command_mock.hpp"

#include "tap/control/command_scheduler_types.hpp"
#include "tap/control/subsystem.hpp"

tap::control::subsystem_scheduler_bitmap_t calcRequirementsBitwise(
    const std::set<tap::control::Subsystem *> subRequirements)
{
    tap::control::subsystem_scheduler_bitmap_t sum = 0;
    for (const auto sub : subRequirements)
    {
        sum |=
            (static_cast<tap::control::subsystem_scheduler_bitmap_t>(1)
             << sub->getGlobalIdentifier());
    }
    return sum;
}

namespace tap::mock
{
CommandMock::CommandMock() : Command()
{
    // Most of the time tests expect that we are adding commands that
    // are ready to be added. This makes tests cleaner
    ON_CALL(*this, isReady).WillByDefault(testing::Return(true));
    ON_CALL(*this, isFinished).WillByDefault(testing::Return(false));
}

CommandMock::CommandMock(const std::set<control::Subsystem *> &subsystemRequirements)
{
    ON_CALL(*this, isReady).WillByDefault(testing::Return(true));
    ON_CALL(*this, isFinished).WillByDefault(testing::Return(false));
    ON_CALL(*this, getRequirementsBitwise)
        .WillByDefault(testing::Return(calcRequirementsBitwise(subsystemRequirements)));
}

CommandMock::~CommandMock() {}
}  // namespace tap::mock
