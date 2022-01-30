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

#include "imu_menu.hpp"

namespace tap::sensors::imu
{
ImuMenu::ImuMenu(
    modm::ViewStack<display::DummyAllocator<modm::IAbstractView> > *stack,
    ImuInterface *imu)
    : modm::AbstractMenu<display::DummyAllocator<modm::IAbstractView> >(stack, 1),
      imu(imu)
{
}

void ImuMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    display.printf("       x     y     z\n");
    // draw accel data
    display.printf(
        "Accel: %.2f %.2f %.2f\n",
        static_cast<double>(imu->getAx()),
        static_cast<double>(imu->getAy()),
        static_cast<double>(imu->getAz()));
    // draw gyro data
    display.printf(
        "Gyro:  %.2f %.2f %.2f\n",
        static_cast<double>(imu->getGx()),
        static_cast<double>(imu->getGy()),
        static_cast<double>(imu->getGz()));
    // draw angle data
    display.printf(
        "Angle: %.2f %.2f %.2f\n",
        static_cast<double>(imu->getPitch()),
        static_cast<double>(imu->getRoll()),
        static_cast<double>(imu->getYaw()));
    // draw temp
    display.printf("Temp:  %.2f", static_cast<double>(imu->getTemp()));
}

void ImuMenu::update() {}

bool ImuMenu::hasChanged() { return imuUpdateTimer.execute(); }

void ImuMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::Button::LEFT)
    {
        this->remove();
    }
}
}  // namespace tap::sensors::imu
