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

#ifndef __ARUW_CLOCK_HPP__
#define __ARUW_CLOCK_HPP__
#include <stdint.h>

#ifndef PLATFORM_HOSTED
#include <modm/platform.hpp>
#else
#include <modm/architecture/interface/clock.hpp>
#endif

namespace aruwlib
{
namespace arch
{
namespace clock
{
inline uint32_t getTimeMilliseconds() { return modm::Clock().now().time_since_epoch().count(); }

/**
 * @warning This clock time will wrap every 72 minutes. Do not use unless absolutely necessary.
 */
inline uint32_t getTimeMicroseconds()
{
    return modm::PreciseClock().now().time_since_epoch().count();
}
}  // namespace clock

}  // namespace arch

}  // namespace aruwlib
#endif
