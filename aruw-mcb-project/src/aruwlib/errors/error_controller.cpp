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

#include "error_controller.hpp"

#include <modm/container/linked_list.hpp>

#include "aruwlib/Drivers.hpp"
#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/communication/gpio/leds.hpp"

namespace aruwlib
{
namespace errors
{
void ErrorController::addToErrorList(const SystemError& error)
{
    if (!validateErrorTypeAndLocation(error))
    {
        return;
    }
    // only add error if it is not already added
    // Note that we are okay with comparing raw char pointers because an error generated
    // in our codebase use char pointers located in literals.
    for (SystemError sysErr : errorList)
    {
        if (sysErr.getErrorType() == error.getErrorType() &&
            sysErr.getLocation() == error.getLocation() &&
            sysErr.getDescription() == error.getDescription() &&
            sysErr.getFilename() == error.getFilename() &&
            sysErr.getLineNumber() == error.getLineNumber())
        {
            return;  // the error is already added
        }
    }
    if (errorList.getSize() >= errorList.getMaxSize())
    {
        errorList.removeFront();  // remove the oldest element in the error list
    }
    errorList.append(error);
}

void ErrorController::updateLedDisplay()
{
    // there are no errors to display, default display
    if (errorList.getSize() == 0)
    {
        displayBinaryNumberViaLeds(0);
        return;
    }

    // change error every ERROR_ROTATE_TIME time increment
    if (prevLedErrorChangeWait.execute())
    {
        prevLedErrorChangeWait.restart(ERROR_ROTATE_TIME);
        currentDisplayIndex = (currentDisplayIndex + 1) % errorList.getSize();

        displayBinaryNumberViaLeds(
            static_cast<uint8_t>(errorList.get(currentDisplayIndex).getLocation()));
    }
}

void ErrorController::displayBinaryNumberViaLeds(uint8_t binaryRep)
{
    // Mask number and determine if it is a 0 or a 1
    // If it is a 1, the LED corresponding will blink
    for (error_index_t i = 0; i < NUM_LEDS; i++)
    {
        bool display = (binaryRep >> i) & 1;
        drivers->leds.set(static_cast<aruwlib::gpio::Leds::LedPin>(i), display);
    }
}

bool ErrorController::validateErrorTypeAndLocation(const SystemError& error)
{
    static constexpr uint8_t locationMask =
        static_cast<uint8_t>(~(0xffu << SystemError::ERROR_LOCATION_SIZE));
    static constexpr uint8_t errorTypeMask =
        static_cast<uint8_t>(~(0xffu << SystemError::ERROR_TYPE_SIZE));

    uint8_t locationMasked = static_cast<uint8_t>(error.getLocation()) & locationMask;
    uint8_t errorTypeMasked = static_cast<uint8_t>(error.getErrorType()) & errorTypeMask;
    return locationMasked == error.getLocation() && errorTypeMasked == error.getErrorType();
}
}  // namespace errors

}  // namespace aruwlib
