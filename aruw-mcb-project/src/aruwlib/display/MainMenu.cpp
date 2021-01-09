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

#include "MainMenu.hpp"

#include "ErrorMenu.hpp"

namespace aruwlib
{
namespace display
{
MainMenu::MainMenu(modm::ViewStack* stack, uint8_t identifier, Drivers* drivers)
    : modm::StandardMenu(stack, identifier),
      drivers(drivers)
{
}

void MainMenu::initialize()
{
    addEntry(
        ErrorMenu::getMenuName(),
        modm::MenuEntryCallback(this, &MainMenu::addErrorMenuCallback));
    addEntry("Motor Menu", modm::MenuEntryCallback(this, &MainMenu::addMotorMenuCallback));
    addEntry(
        "Property Table Menu",
        modm::MenuEntryCallback(this, &MainMenu::addPropertyTableCallback));
    setTitle("Main Menu");
}

void MainMenu::addErrorMenuCallback()
{
    getViewStack()->push(new ErrorMenu(getViewStack(), drivers));
}

void MainMenu::addMotorMenuCallback()
{
    // TODO, see issue #105
}

void MainMenu::addPropertyTableCallback()
{
    // TODO, see issue #221
}
}  // namespace display

}  // namespace aruwlib
