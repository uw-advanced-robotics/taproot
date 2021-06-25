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

#ifndef ERROR_MENU_HPP_
#define ERROR_MENU_HPP_

#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwlib
{
class Drivers;
namespace display
{
class ErrorMenu : public modm::AbstractMenu
{
public:
    ErrorMenu(modm::ViewStack *vs, Drivers *drivers);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Error Menu"; }

private:
    static constexpr int ERROR_MENU_ID = 3;

    Drivers *drivers;
};  // class ErrorMenu
}  // namespace display
}  // namespace aruwlib

#endif  // ERROR_MENU_HPP_
