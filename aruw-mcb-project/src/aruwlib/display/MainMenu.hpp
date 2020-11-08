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

#ifndef MAIN_MENU_HPP_
#define MAIN_MENU_HPP_

#include <modm/ui/menu/standard_menu.hpp>

namespace aruwlib
{
class Drivers;
namespace display
{
class MainMenu : public modm::StandardMenu
{
public:
    MainMenu(modm::ViewStack *stack, uint8_t identifier, Drivers *drivers);

    virtual ~MainMenu() = default;

    /**
     * Adds entries to the menu to the necessary submenus.
     */
    void initialize();

private:
    Drivers *drivers;

    void addErrorMenuCallback();
    void addMotorMenuCallback();
    void addPropertyTableCallback();
};  // class MainMenu
}  // namespace display
}  // namespace aruwlib

#endif  // MAIN_MENU_HPP_
