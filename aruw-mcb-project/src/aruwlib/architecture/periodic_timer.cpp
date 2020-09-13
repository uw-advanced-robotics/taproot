/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "periodic_timer.hpp"

#include "aruwlib/architecture/clock.hpp"

using namespace aruwlib::arch;

PeriodicMilliTimer::PeriodicMilliTimer() : period(0) {}

PeriodicMilliTimer::PeriodicMilliTimer(uint32_t period) : period(period), timeout(period) {}

void PeriodicMilliTimer::restart() { timeout.restart(period); }

void PeriodicMilliTimer::restart(uint32_t period)
{
    this->period = period;
    restart();
}

void PeriodicMilliTimer::stop() { timeout.stop(); }

bool PeriodicMilliTimer::isStopped() const { return timeout.isStopped(); }

bool PeriodicMilliTimer::execute()
{
    if (timeout.execute())
    {
        uint32_t now = clock::getTimeMilliseconds();

        do
        {
            timeout.expireTime += period;
        } while (timeout.expireTime <= now);

        timeout.isRunning = true;
        timeout.isExecuted = false;
        return true;
    }
    return false;
}
