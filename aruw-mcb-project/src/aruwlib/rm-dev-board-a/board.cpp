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

#include "board.hpp"

// In simulation, we'll let modm's default implementation handle this.
#ifndef ENV_SIMULATOR
modm_extern_c void modm_abandon(const char *, const char *, const char *, uintptr_t)
{
    Board::LedsPort::setOutput();
    for (int times = 10; times >= 0; times--)
    {
        Board::LedsPort::toggle();
        modm::delayMilliseconds(100);
        Board::LedsPort::toggle();
        modm::delayMilliseconds(100);
    }
}

#endif
