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

#ifndef REF_SERIAL_UI_WRAPPERS_HPP_
#define REF_SERIAL_UI_WRAPPERS_HPP_

#include "tap/architecture/timeout.hpp"
#include "../ref_serial.hpp"
#include "modm/processing/resumable.hpp"

namespace tap{
    class Drivers;
}

namespace tap::communication::serial
{
class UiWrapper
{
    virtual modm::ResumableResult<bool> initialize() = 0;
    virtual modm::ResumableResult<bool> draw() = 0;
};

/**
 * Draws a bubble that is either filled in or not on the UI
 */
class BubbleDrawer : public modm::Resumable<2>
{
public:
    BubbleDrawer(tap::Drivers *drivers, tap::serial::RefSerial::Tx::Graphic1Message *bubbleMessage);

    modm::ResumableResult<bool> initialize();
    modm::ResumableResult<bool> draw();

    void setBubbleFilled(bool filled);

private:
    tap::Drivers *drivers;

    tap::serial::RefSerial::Tx::Graphic1Message* bubble;
    tap::serial::RefSerial::Tx::GraphicColor savedColor = tap::serial::RefSerial::Tx::GraphicColor::WHITE;

    bool filled = false;
    bool colorChanged = false;

    tap::arch::MilliTimeout delayTimeout;

    void updateBubbleColor();
};
}  // namespace tap::communication::serial

#endif  // REF_SERIAL_UI_WRAPPERS_HPP_
