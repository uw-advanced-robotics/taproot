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

#include "bubble_drawer.hpp"

#include "tap/drivers.hpp"

using namespace tap::serial;

const uint32_t GRAPHIC_SEND_PERIOD = 10;

#define delay()                                \
    delayTimeout.restart(GRAPHIC_SEND_PERIOD); \
    RF_WAIT_UNTIL(delayTimeout.execute());

namespace tap::communication::serial
{
BubbleDrawer::BubbleDrawer(
    tap::Drivers *drivers,
    tap::serial::RefSerial::Tx::Graphic1Message *bubbleMessage)
    : drivers(drivers),
      bubble(bubbleMessage)
{
}

modm::ResumableResult<bool> BubbleDrawer::initialize()
{
    RF_BEGIN(0);
    savedColor = static_cast<tap::serial::RefSerial::Tx::GraphicColor>(bubble->graphicData.color);
    updateBubbleColor();
    // Initially add the graphic
    bubble->graphicData.operation = tap::serial::RefSerial::Tx::ADD_GRAPHIC;
    drivers->refSerial.sendGraphic(bubble);
    // In future calls to sendGraphic only modify the graphic
    bubble->graphicData.operation = tap::serial::RefSerial::Tx::ADD_GRAPHIC_MODIFY;
    delay();
    RF_END();
}

modm::ResumableResult<bool> BubbleDrawer::draw()
{
    RF_BEGIN(1);
    updateBubbleColor();
    if (colorChanged)
    {
        // resend graphic if color changed
        drivers->refSerial.sendGraphic(bubble);
        colorChanged = false;
        delay();
    }
    RF_END();
}

void BubbleDrawer::setBubbleFilled(bool filled) { this->filled = filled; }

void BubbleDrawer::updateBubbleColor()
{
    uint32_t prevColor = bubble->graphicData.color;
    tap::serial::RefSerial::Tx::GraphicColor color;
    color = filled ? savedColor : tap::serial::RefSerial::Tx::GraphicColor::WHITE;
    bubble->graphicData.color = static_cast<uint32_t>(color) & 0b1111;
    colorChanged = prevColor != bubble->graphicData.color;
}
}  // namespace tap::communication::serial
