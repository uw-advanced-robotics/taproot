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

#ifndef ERROR_CONTROLLER_HPP
#define ERROR_CONTROLLER_HPP

#include <aruwlib/architecture/timeout.hpp>
#include <modm/container.hpp>

#include "mock_macros.hpp"
#include "system_error.hpp"

namespace aruwlib
{
class Drivers;
namespace errors
{
/**
 * Protocol description:
 * The 8 leds on the mcb are used to indicate a location and error type. LEDs A-D transmit
 * the error location and E-H the error type.
 * For location, the LSB is D, and for error type, the LSB is H
 * The other green led (next to the red led) comes on
 * when you have added an invalid error. The red led is always on (not used). Default, leds
 * A-H are always off if no errors are detected
 */
class ErrorController
{
public:
    ErrorController(Drivers* drivers) : drivers(drivers), prevLedErrorChangeWait(ERROR_ROTATE_TIME)
    {
    }
    ErrorController(const ErrorController&) = delete;
    ErrorController& operator=(const ErrorController&) = delete;
    mockable ~ErrorController() = default;

    mockable void addToErrorList(const SystemError& error);

    mockable void update();

private:
    static const int ERROR_ROTATE_TIME = 5000;

    static const unsigned ERROR_LIST_MAX_SIZE = 16;

    Drivers* drivers;

    modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE> errorList;

    aruwlib::arch::MilliTimeout prevLedErrorChangeWait;

    int currentDisplayIndex = 0;

    bool getLedErrorCodeBits(Location location, ErrorType errorType, uint8_t* number);

    void setLedError(uint8_t binaryRep);

    void ledSwitch(uint8_t ledOnBoard, bool displayOnBoard);
};

}  // namespace errors

}  // namespace aruwlib

#endif
