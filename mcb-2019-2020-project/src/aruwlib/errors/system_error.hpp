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
    CAN_RX = 0,
    MOTOR_CONTROL,
    MPU6500,
    DJI_SERIAL,
    COMMAND_SCHEDULER,
    LOCATION_AMOUNT
};

// Type of errors; subject to change
enum ErrorType {
    IMU_DATA_NOT_INITIALIZED = 0,
    IMU_NOT_RECEIVING_PROPERLY,
    INVALID_MESSAGE_LENGTH,
    NULL_MOTOR_ID,
    CRC8_FAILURE,
    CRC16_FAILURE,
    MESSAGE_LENGTH_OVERFLOW,
    RUN_TIME_OVERFLOW,
    MOTOR_ID_OUT_OF_BOUNDS,
    ADDING_NULLPTR_COMMAND,
    ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES,
    ERROR_TYPE_AMOUNT
};

class SystemError
{
 public:
    SystemError() : location(LOCATION_AMOUNT), errorType(ERROR_TYPE_AMOUNT) {}

    SystemError(Location l, ErrorType et) : location(l), errorType(et)
    {
        static_assert(LOCATION_AMOUNT <= ERROR_LOCATION_SIZE * ERROR_LOCATION_SIZE,
            "You have declared too many locations!");
        static_assert(ERROR_TYPE_AMOUNT <= ERROR_TYPE_SIZE * ERROR_TYPE_SIZE,
            "You have declared too many error types!");
    }

    Location getLocation() const;

    ErrorType getErrorType() const;

 private:
    Location location;

    ErrorType errorType;
};

}  // namespace errors

}  // namespace aruwlib

#endif
