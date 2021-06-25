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

#ifndef SPLASH_SCREEN_HPP_
#define SPLASH_SCREEN_HPP_

#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwlib
{
class Drivers;
namespace display
{
class SplashScreen : public modm::AbstractMenu
{
public:
    SplashScreen(modm::ViewStack *vs, aruwlib::Drivers *drivers);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    inline void resetHasChanged() { drawn = false; }

private:
    static constexpr int SPLASH_SCREEN_MENU_ID = 1;

    bool drawn = false;
    aruwlib::Drivers *drivers;
};
}  // namespace display
}  // namespace aruwlib

#endif  // SPLASH_SCREEN_HPP_
