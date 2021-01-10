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

#ifndef HARDWARE_TEST_MENU_HPP_
#define HARDWARE_TEST_MENU_HPP_

#include <modm/ui/menu/abstract_menu.hpp>

#include "aruwlib/Drivers.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"

#include "modm/processing/timer/periodic_timer.hpp"

namespace aruwlib
{
namespace display
{
class HardwareTestMenu : public modm::AbstractMenu
{
public:
    HardwareTestMenu(modm::ViewStack *vs, Drivers *drivers);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Hardware Test Menu"; }

private:
    Drivers *drivers;

    int selectedSubsystem = 0;

    int bottomIndex = 0;
    int topIndex = 8;  // TODO: unhardcode this eventually

    bool changed = false;

    uint64_t completeSubsystems = 0;

    bool updateHasChanged();
};  // class HardwareTestMenu
}  // namespace display
}  // namespace aruwlib

#endif  // HARDWARE_TEST_MENU_HPP_
