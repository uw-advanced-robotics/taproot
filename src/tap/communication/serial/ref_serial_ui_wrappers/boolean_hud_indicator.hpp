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

#ifndef BOOLEAN_HUD_INDICATOR_HPP_
#define BOOLEAN_HUD_INDICATOR_HPP_

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
 * Draws a graphic. Each graphic has an associated true/false state. When the state associated with
 * this object is updated, a function is called in which the user can update the graphic based on
 * the state. In this way, the user may select to update color or shape of a graphic based on the
 * indicator's state.
 *
 * To use this drawer, pass in a `Graphic1Message` object that you must configure via calls to
 * `configGraphicGenerics` and `config<circle|line|etc.>`.
 *
 * @note `initialize` should be called once at the start of a protothread being used to write
 *      graphic information to the referee system and `draw` should be called repeatedly, and will
 *      either resend the graphic if the color has changed or do nothing.
 *
 * Usage:
 *
 * To construct some `BooleanHUDIndicator indicator`
 * you must pass it an `UpdateHUDIndicatorState` function. For example, the function can be
 * something like this:
 *
 * ```
 * void updateColor(bool state, Tx::Graphic1Message *graphic) {
 *     graphic->graphicData.color = indicatorStatus ? GREEN : YELLOW;
 * }
 * ```
 *
 * Assuming some `Tx::Graphic1Message graphic` has been declared, to use the `indicator`, calls to
 * `initialize` and `draw` must be done in a prothread similar to the following. It is recommended
 * you put `PT_YIELD` in protothread loops to avoid infinite looping since the indicator's `draw`
 * function is not guaranteed to block:
 *
 * ```
 * PT_CALL(indicator.initialize());
 *
 * while (true)
 * {
 *     PT_CALL(indicator.draw());
 *     PT_YIELD();
 * }
 * // Initialize the declare a drawer
 * ```
 */
class BooleanHUDIndicator : public modm::Resumable<2>
{
public:
    /**
     * Function pointer, this type of function will be called when the state of the graphic needs
     * updating. Expected that the user will update the graphic appropriately based on the current
     * state.
     */
    using UpdateHUDIndicatorState =
        void (*)(bool state, tap::serial::RefSerial::Tx::Graphic1Message *graphic);

    /**
     * The boolean will ignore calls in `setIndicatorState` MIN_UPDATE_PERIOD ms after
     * `setIndicatorState` is drawn. This is to avoid rapid back and forth updating of the graphic.
     */
    static constexpr uint32_t MIN_UPDATE_PERIOD = 500;

    BooleanHUDIndicator(
        tap::Drivers *drivers,
        tap::serial::RefSerial::Tx::Graphic1Message *graphic,
        UpdateHUDIndicatorState updateFunction);

    modm::ResumableResult<bool> initialize();
    modm::ResumableResult<bool> draw();

    void setIndicatorState(bool newIndicatorState);

private:
    tap::Drivers *drivers;

    tap::serial::RefSerial::Tx::Graphic1Message *graphic;

    UpdateHUDIndicatorState updateFunction;

    bool indicatorState = false;
    bool indicatorChanged = false;

    tap::arch::MilliTimeout delayTimeout;

    tap::arch::MilliTimeout minUpdatePeriodTimeout;
};
}  // namespace tap::communication::serial::ref_serial_ui_wrapeprs

#endif  // BOOLEAN_HUD_INDICATOR_HPP_
