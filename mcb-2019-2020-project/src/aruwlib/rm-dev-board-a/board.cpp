/*
 * Copyright (c) 2016-2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

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
