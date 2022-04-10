/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

namespace tap::communication::sensors::imu
{
ImuMenu::ImuMenu(
    modm::ViewStack<display::DummyAllocator<modm::IAbstractView> > *stack,
    ImuInterface *imu)
    : modm::AbstractMenu<display::DummyAllocator<modm::IAbstractView> >(stack, 1),
      imu(imu),
      imuAccelGyroAngleFnPtrs{
          {&ImuInterface::getAx, &ImuInterface::getAy, &ImuInterface::getAz},
          {&ImuInterface::getGx, &ImuInterface::getGy, &ImuInterface::getGz},
          {&ImuInterface::getPitch, &ImuInterface::getRoll, &ImuInterface::getYaw},
      }
{
}

static void drawFloat(modm::GraphicDisplay &display, float val)
{
    display.printf("%.2f", static_cast<double>(val));
}

void ImuMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    // print row headers and temperature
    display.printf("\nAcc\nGyro\nAng\nTemp:  %.2f", static_cast<double>(imu->getTemp()));

    for (size_t x = 0; x < MODM_ARRAY_SIZE(imuAccelGyroAngleFnPtrs[0]); x++)
    {
        // print the column title
        display.setCursor(
            IMU_DATA_START_X + x * (display.getBufferWidth() - IMU_DATA_START_X) /
                                   MODM_ARRAY_SIZE(imuAccelGyroAngleFnPtrs[0]),
            IMU_DATA_START_Y);
        display.printf("%s", IMU_DATA_COL_HEADERS[x]);

        // print column
        for (size_t y = 0; y < MODM_ARRAY_SIZE(imuAccelGyroAngleFnPtrs); y++)
        {
            display.setCursor(
                IMU_DATA_START_X + x * (display.getBufferWidth() - IMU_DATA_START_X) /
                                       MODM_ARRAY_SIZE(imuAccelGyroAngleFnPtrs[0]),
                IMU_DATA_START_Y + (y + 1) * display.getFontHeight());
            drawFloat(display, (imu->*imuAccelGyroAngleFnPtrs[y][x])());
        }
    }
}

void ImuMenu::update() {}

bool ImuMenu::hasChanged() { return imuUpdateTimer.execute(); }

void ImuMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    // when left button pressed, remove the IMU menu
    if (button == modm::MenuButtons::Button::LEFT)
    {
        this->remove();
    }
}

const char *ImuMenu::getMenuName() { return imu->getName(); }

}  // namespace tap::communication::sensors::imu
