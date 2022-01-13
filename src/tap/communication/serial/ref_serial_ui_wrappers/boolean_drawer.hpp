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

#ifndef REF_SERIAL_UI_WRAPPERS_HPP_
#define REF_SERIAL_UI_WRAPPERS_HPP_

#include "tap/architecture/timeout.hpp"

#include "../ref_serial.hpp"
#include "modm/processing/resumable.hpp"

namespace tap
{
class Drivers;
}

namespace tap::communication::serial::ref_serial_ui_wrapeprs
{
/**
 * Draws a graphic representation of a boolean value, using color to indicate the state of the
 * boolean variable associated with the graphic. To use this drawer, pass in a `Graphic1Message`
 * object that you must configure via calls to `configGraphicGenerics` and
 * `config<circle|line|etc.>`. before you call this function's `initialize` function. The initial
 * color you specify when calling `configGraphicGenerics` will be the color of the graphic when the
 * boolean associated with the drawer is `true`, and the color of the graphic when the boolean
 * associated with the drawer is `false` can be specified in the constructor.
 *
 * @note `initialize` should be called once at the start of a protothread being used to write
 *      graphic information to the referee system and `draw` should be called repeatedly, and will
 *      either resend the graphic if the color has changed or do nothing.
 */
class BooleanDrawer : public modm::Resumable<2>
{
public:
    BooleanDrawer(
        tap::Drivers *drivers,
        tap::serial::RefSerial::Tx::Graphic1Message *graphic,
        tap::serial::RefSerial::Tx::GraphicColor boolFalseColor =
            tap::serial::RefSerial::Tx::GraphicColor::WHITE);

    modm::ResumableResult<bool> initialize();
    modm::ResumableResult<bool> draw();

    void setDrawerColor(bool filledWithInitialColor);

    void setBoolFalseColor(tap::serial::RefSerial::Tx::GraphicColor boolFalseColor)
    {
        this->boolFalseColor = boolFalseColor;
    }

private:
    tap::Drivers *drivers;

    tap::serial::RefSerial::Tx::Graphic1Message *graphic;
    tap::serial::RefSerial::Tx::GraphicColor savedColor =
        tap::serial::RefSerial::Tx::GraphicColor::WHITE;
    tap::serial::RefSerial::Tx::GraphicColor boolFalseColor;

    bool filledWithInitialColor = false;
    bool colorChanged = false;

    tap::arch::MilliTimeout delayTimeout;

    void updateColor();
};
}  // namespace tap::communication::serial::ref_serial_ui_wrapeprs

#endif  // REF_SERIAL_UI_WRAPPERS_HPP_
