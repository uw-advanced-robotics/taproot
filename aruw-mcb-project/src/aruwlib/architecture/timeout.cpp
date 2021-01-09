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

#include "timeout.hpp"

#include <aruwlib/architecture/clock.hpp>

namespace aruwlib
{
namespace arch
{
MilliTimeout::MilliTimeout()
{
    stop();
    this->expireTime = 0;
}

MilliTimeout::MilliTimeout(uint32_t timeout_millis) { restart(timeout_millis); }

void MilliTimeout::restart(uint32_t timeout_millis)
{
    this->isRunning = true;
    this->isExecuted = false;
    this->expireTime = clock::getTimeMilliseconds() + timeout_millis;
}

void MilliTimeout::stop()
{
    this->isRunning = false;
    this->isExecuted = false;
}

bool MilliTimeout::isExpired() const
{
    return this->isRunning && clock::getTimeMilliseconds() >= this->expireTime;
}

bool MilliTimeout::isStopped() const { return !this->isRunning; }

bool MilliTimeout::execute()
{
    if (!isExecuted && isExpired())
    {
        isExecuted = true;
        return true;
    }

    return false;
}
}  // namespace arch
}  // namespace aruwlib
