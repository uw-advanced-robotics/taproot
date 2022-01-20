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

#include "boolean_hud_indicator.hpp"

#include "tap/drivers.hpp"

using namespace tap::serial;

#define delay()                                                                      \
    delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(graphic)); \
    RF_WAIT_UNTIL(delayTimeout.execute());

namespace tap::communication::serial::ref_serial_ui_wrapeprs
{
BooleanHUDIndicator::BooleanHUDIndicator(
    tap::Drivers *drivers,
    RefSerial::Tx::Graphic1Message *graphic,
    UpdateHUDIndicatorState updateFunction)
    : drivers(drivers),
      graphic(graphic),
      updateFunction(updateFunction)
{
    minUpdatePeriodTimeout.stop();
}

modm::ResumableResult<bool> BooleanHUDIndicator::initialize()
{
    RF_BEGIN(0);
    // Initially add the graphic
    graphic->graphicData.operation = tap::serial::RefSerial::Tx::ADD_GRAPHIC;
    drivers->refSerial.sendGraphic(graphic);
    // In future calls to sendGraphic only modify the graphic
    graphic->graphicData.operation = tap::serial::RefSerial::Tx::ADD_GRAPHIC_MODIFY;
    delay();
    RF_END();
}

modm::ResumableResult<bool> BooleanHUDIndicator::draw()
{
    RF_BEGIN(1);
    if (indicatorChanged)
    {
        // resend graphic if color changed
        drivers->refSerial.sendGraphic(graphic);
        indicatorChanged = false;
        delay();
    }
    RF_END();
}

void BooleanHUDIndicator::setIndicatorState(bool newIndicatorState)
{
    if (minUpdatePeriodTimeout.isExpired() || minUpdatePeriodTimeout.isStopped())
    {
        if (indicatorState != newIndicatorState)
        {
            indicatorState = newIndicatorState;
            updateFunction(indicatorState, graphic);
            indicatorChanged = true;
            minUpdatePeriodTimeout.restart(MIN_UPDATE_PERIOD);
        }
    }
}

}  // namespace tap::communication::serial::ref_serial_ui_wrapeprs
