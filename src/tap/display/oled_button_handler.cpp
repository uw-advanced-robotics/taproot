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

#include "oled_button_handler.hpp"

#include "tap/drivers.hpp"

namespace tap
{
namespace display
{
OledButtonHandler::OledButtonHandler(tap::Drivers *drivers, const AnalogConfig analogConfig)
    : drivers(drivers),
      downButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      upButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      leftButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      rightButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      okButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      adcConfig(analogConfig)
{
}

OledButtonHandler::Button OledButtonHandler::getCurrentButtonState()
{
    int buttonADC = drivers->analog.read(gpio::Analog::Pin::OledJoystick);

    downButtonPressed.update(abs(buttonADC - adcConfig.down) < ADC_PRESSED_RANGE);
    upButtonPressed.update(abs(buttonADC - adcConfig.up) < ADC_PRESSED_RANGE);
    leftButtonPressed.update(abs(buttonADC - adcConfig.left) < ADC_PRESSED_RANGE);
    rightButtonPressed.update(abs(buttonADC - adcConfig.right) < ADC_PRESSED_RANGE);
    okButtonPressed.update(abs(buttonADC - adcConfig.ok) < ADC_PRESSED_RANGE);

    if (downButtonPressed.getValue())
    {
        return Button::DOWN;
    }
    else if (upButtonPressed.getValue())
    {
        return Button::UP;
    }
    else if (rightButtonPressed.getValue())
    {
        return Button::RIGHT;
    }
    else if (leftButtonPressed.getValue())
    {
        return Button::LEFT;
    }
    else if (okButtonPressed.getValue())
    {
        return Button::OK;
    }
    else
    {
        return Button::NONE;
    }
}
}  // namespace display
}  // namespace tap
