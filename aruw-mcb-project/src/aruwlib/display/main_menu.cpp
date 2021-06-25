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

#include "main_menu.hpp"

#include "command_scheduler_menu.hpp"
#include "error_menu.hpp"
#include "hardware_test_menu.hpp"
#include "motor_menu.hpp"

namespace aruwlib
{
namespace display
{
MainMenu::MainMenu(modm::ViewStack* stack, Drivers* drivers)
    : modm::StandardMenu(stack, MAIN_MENU_ID),
      drivers(drivers)
{
}

void MainMenu::initialize()
{
    addEntry(
        ErrorMenu::getMenuName(),
        modm::MenuEntryCallback(this, &MainMenu::addErrorMenuCallback));

    addEntry(
        HardwareTestMenu::getMenuName(),
        modm::MenuEntryCallback(this, &MainMenu::addHardwareTestMenuCallback));

    addEntry(
        MotorMenu::getMenuName(),
        modm::MenuEntryCallback(this, &MainMenu::addMotorMenuCallback));

    addEntry(
        "Property Table Menu",
        modm::MenuEntryCallback(this, &MainMenu::addPropertyTableCallback));

    addEntry(
        CommandSchedulerMenu::getMenuName(),
        modm::MenuEntryCallback(this, &MainMenu::addCommandSchedulerCallback));

    setTitle("Main Menu");
}

void MainMenu::addErrorMenuCallback()
{
    getViewStack()->push(new ErrorMenu(getViewStack(), drivers));
}

void MainMenu::addHardwareTestMenuCallback()
{
    getViewStack()->push(new HardwareTestMenu(getViewStack(), drivers));
}

void MainMenu::addMotorMenuCallback()
{
    getViewStack()->push(new MotorMenu(getViewStack(), drivers));
}

void MainMenu::addPropertyTableCallback()
{
    // TODO, see issue #221
}

void MainMenu::addCommandSchedulerCallback()
{
    getViewStack()->push(new CommandSchedulerMenu(getViewStack(), drivers));
}
}  // namespace display

}  // namespace aruwlib
