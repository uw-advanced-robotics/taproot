#ifndef ERROR_CONTROLLER_HPP
#define ERROR_CONTROLLER_HPP

#include <modm/container.hpp>
#include <modm/processing/timer.hpp>
#include <rm-dev-board-a/board.hpp>
#include "system_error.hpp"

namespace aruwlib
{

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
    static void addToErrorList(SystemError error);

    static void update();

 private:
    static const int ERROR_ROTATE_TIME = 3000;

    static const unsigned ERROR_LIST_MAX_SIZE = 16;

    static modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE> errorList;

    static modm::ShortTimeout prevLedErrorChangeWait;

    static int currentDisplayIndex;

    static bool getLedErrorCodeBits(Location location, ErrorType errorType, uint8_t* number);

    static void setLedError(uint8_t binaryRep);

    static void ledSwitch(uint8_t ledLocation, bool display);
};

}  // namespace errors

}  // namespace aruwlib

#endif
