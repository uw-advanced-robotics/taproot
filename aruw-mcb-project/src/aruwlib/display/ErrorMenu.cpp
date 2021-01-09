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

#include "ErrorMenu.hpp"

namespace aruwlib
{
namespace display
{
ErrorMenu::ErrorMenu(modm::ViewStack *vs, Drivers *drivers) : AbstractMenu(vs, 1), drivers(drivers)
{
}

void ErrorMenu::update()
{
    if (this->hasChanged())
    {
        this->draw();
    }
}

void ErrorMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->getViewStack()->pop();
            break;
        case modm::MenuButtons::RIGHT:
            break;
        case modm::MenuButtons::DOWN:
            break;
        case modm::MenuButtons::UP:
            break;
        case modm::MenuButtons::OK:
            break;
    }
}

bool ErrorMenu::hasChanged()
{
    // TODO implement, see issue #222
    // This should return true only when the state of the ErrorMenu has changed
    // (the stuff on the display has changed) to minimize I/O usage.
    return true;
}

void ErrorMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << ErrorMenu::getMenuName();
    // TODO implement, see issue #222
}
}  // namespace display
}  // namespace aruwlib
