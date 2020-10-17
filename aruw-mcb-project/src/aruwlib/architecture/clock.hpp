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
inline uint32_t getTimeMilliseconds() { return modm::Clock::now().getTime(); }

inline uint32_t getTimeMicroseconds()
{
#ifdef PLATFORM_HOSTED
    return 0;
#else
    return DWT->CYCCNT / static_cast<uint32_t>(modm::clock::fcpu_MHz);
#endif
}
}  // namespace clock

}  // namespace arch

}  // namespace aruwlib
#endif
