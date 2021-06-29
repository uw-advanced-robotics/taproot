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

#include "oled_display.hpp"

#include "aruwlib/drivers.hpp"

using namespace aruwlib::display;
using namespace modm::literals;

namespace aruwsrc
{
namespace display
{
OledDisplay::OledDisplay(aruwlib::Drivers *drivers)
    : display(),
      viewStack(&display),
      buttonHandler(drivers),
      splashScreen(&viewStack, drivers),
      drivers(drivers)
{
}

void OledDisplay::initialize()
{
#ifndef PLATFORM_HOSTED
    Board::DisplaySpiMaster::
        connect<Board::DisplayMiso::Miso, Board::DisplayMosi::Mosi, Board::DisplaySck::Sck>();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    Board::DisplaySpiMaster::initialize<Board::SystemClock, 1406250_Hz>();
#endif

    display.initializeBlocking();
    display.setFont(modm::font::ScriptoNarrow);

    viewStack.push(&splashScreen);
}

bool OledDisplay::updateDisplay()
{
    PT_BEGIN();
    while (true)
    {
        PT_CALL(display.updateNonblocking());
        PT_WAIT_UNTIL(displayThreadTimer.execute());
    }
    PT_END();
    return false;
}

void OledDisplay::updateMenu()
{
    OledButtonHandler::Button buttonPressed = buttonHandler.getCurrentButtonState();
    if (buttonPressed != OledButtonHandler::NONE && buttonPressed != prevButton)
    {
        // Seperate from above for ease of readability.
        // For now the main menu should never be removed from the stack (which is what the left
        // button does in a StandardMenu).
        if (buttonPressed != OledButtonHandler::LEFT || viewStack.get() != &splashScreen)
        {
            viewStack.shortButtonPress(static_cast<modm::MenuButtons::Button>(buttonPressed));
        }
    }
    prevButton = buttonPressed;

    viewStack.update();
}
}  // namespace display
}  // namespace aruwsrc
