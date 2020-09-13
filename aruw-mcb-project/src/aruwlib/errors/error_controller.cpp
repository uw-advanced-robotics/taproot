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

#include "error_controller.hpp"

#include <modm/container/linked_list.hpp>

#include "aruwlib/Drivers.hpp"
#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/communication/gpio/leds.hpp"

// Overall method to use when receiving errors

namespace aruwlib
{
namespace errors
{
// add an error to list of errors
void ErrorController::addToErrorList(const SystemError& error)
{
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

// Blink the list of errors in a loop on the board
void ErrorController::update()
{
    // there are no errors to display, default display
    if (errorList.getSize() == 0)
    {
        setLedError(0);
        drivers->leds.set(aruwlib::gpio::Leds::LedPin::Green, true);
        return;
    }

    // change error every ERROR_ROTATE_TIME time increment
    if (prevLedErrorChangeWait.execute())
    {
        prevLedErrorChangeWait.restart(ERROR_ROTATE_TIME);
        currentDisplayIndex = (currentDisplayIndex + 1) % errorList.getSize();
    }

    uint8_t displayNum = 0;
    if (getLedErrorCodeBits(
            errorList.get(currentDisplayIndex).getLocation(),
            errorList.get(currentDisplayIndex).getErrorType(),
            &displayNum))
    {
        setLedError(displayNum);
        drivers->leds.set(aruwlib::gpio::Leds::LedPin::Green, true);
    }
    else
    {
        setLedError(0);
        drivers->leds.set(aruwlib::gpio::Leds::LedPin::Green, false);
    }
}

bool ErrorController::getLedErrorCodeBits(Location location, ErrorType errorType, uint8_t* number)
{
    // Limit location and error type
    // Check to make sure they are within bounds

    // find the bit mask for the location
    uint8_t locationMask = static_cast<uint8_t>(~(0xffu << ERROR_LOCATION_SIZE));
    uint8_t errorTypeMask = static_cast<uint8_t>(~(0xffu << ERROR_TYPE_SIZE));

    uint8_t locationMasked = static_cast<uint8_t>(location) & locationMask;
    uint8_t errorTypeMasked = static_cast<uint8_t>(errorType) & errorTypeMask;

    // set another error if this error is outside of the range of the error handler
    if (locationMasked != location || errorTypeMasked != errorType)
    {
        return false;
    }

    // Combine location and error type
    *number = location << ERROR_LOCATION_SIZE | errorType;
    return true;
}

void ErrorController::setLedError(uint8_t binaryRep)
{
    // Mask number and determine if it is a 0 or a 1
    // If it is a 1, the LED corresponding will blink
    for (int i = 0; i < 8; i++)
    {
        bool display = (binaryRep >> i) & 1;
        drivers->leds.set(static_cast<aruwlib::gpio::Leds::LedPin>(i), display);
    }
}
}  // namespace errors

}  // namespace aruwlib
