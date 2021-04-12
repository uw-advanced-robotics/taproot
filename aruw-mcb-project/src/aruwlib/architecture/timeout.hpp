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

#ifndef TIMEOUT_HPP
#define TIMEOUT_HPP

#include <stdint.h>

#include "clock.hpp"

namespace aruwlib
{
namespace arch
{
template <uint32_t (*T)()>
class Timeout
{
    template <typename H>
    friend class PeriodicTimer;

private:
    bool isRunning;
    bool isExecuted;
    uint32_t expireTime;

public:
    static constexpr auto TimeFunc = T;

    Timeout()
    {
        stop();
        this->expireTime = 0;
    }

    explicit Timeout(uint32_t timeout) { restart(timeout); }

    void restart(uint32_t timeout)
    {
        this->isRunning = true;
        this->isExecuted = false;
        this->expireTime = TimeFunc() + timeout;
    }

    void stop()
    {
        this->isRunning = false;
        this->isExecuted = false;
    }

    bool isStopped() const { return !this->isRunning; }

    bool isExpired() const { return this->isRunning && TimeFunc() >= this->expireTime; }

    bool execute()
    {
        if (!isExecuted && isExpired())
        {
            isExecuted = true;
            return true;
        }

        return false;
    }
};

using MicroTimeout = Timeout<aruwlib::arch::clock::getTimeMicroseconds>;
using MilliTimeout = Timeout<aruwlib::arch::clock::getTimeMilliseconds>;
}  // namespace arch
}  // namespace aruwlib

#endif
