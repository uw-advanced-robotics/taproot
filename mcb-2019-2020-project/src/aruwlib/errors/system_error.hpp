/**
 * example for how to create and add an error to the ErrorController:
 * 
 *     SystemError error1(MOTOR_CONTROL, MOTOR_DISCONNECTED);
 *     ErrorController::addToErrorList(error1);
 * 
 * then call ErrorController::update() to update the list of errors
 */

#ifndef LED_ERROR_HPP
#define LED_ERROR_HPP

#include <rm-dev-board-a/board.hpp>

namespace aruwlib
{

namespace errors
{

static const uint8_t ERROR_LOCATION_SIZE = 4;

static const uint8_t ERROR_TYPE_SIZE = 8 - ERROR_LOCATION_SIZE;

// Location of errors; subject to change
enum Location {
    CAN = 0, MOTOR_CONTROL, LOCATION_AMOUNT
};

// Type of errors; subject to change
enum ErrorType {
    MOTOR_DISCONNECTED = 0, BAD_MOTOR_INPUT,
    ERROR_TYPE_AMOUNT
};

class SystemError
{
 public:
    SystemError(Location l, ErrorType et) : location(l), errorType(et)
    {
        static_assert(LOCATION_AMOUNT <= ERROR_LOCATION_SIZE * ERROR_LOCATION_SIZE,
            "You have declared too many locations!");
        static_assert(ERROR_TYPE_AMOUNT <= ERROR_TYPE_SIZE * ERROR_TYPE_SIZE,
            "You have declared too many error types!");
    }

    Location getLocation(void) const;

    ErrorType getErrorType(void) const;

 private:
    Location location;

    ErrorType errorType;
};

}  // namespace errors

}  // namespace aruwlib

#endif
