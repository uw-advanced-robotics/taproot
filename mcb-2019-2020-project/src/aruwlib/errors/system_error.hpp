/**
 * example for how to create and add an error to the ErrorController:
 * use macro in create_errors.hpp
 *
 *     RAISE_ERROR("Error in DJI Serial", aruwlib::errors::Location::DJI_SERIAL,
 *     aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
 *
 * then call ErrorController::update() to update the list of errors
 */

#ifndef __SYSTEM_ERROR_HPP__
#define __SYSTEM_ERROR_HPP__

#include <string>

namespace aruwlib
{
namespace errors
{
static const uint8_t ERROR_LOCATION_SIZE = 4;

static const uint8_t ERROR_TYPE_SIZE = 8 - ERROR_LOCATION_SIZE;

// Location of errors; subject to change
enum Location
{
    CAN_RX = 0,
    MOTOR_CONTROL,
    MPU6500,
    DJI_SERIAL,
    COMMAND_SCHEDULER,
    SUBSYSTEM,
    CONTROLLER_MAPPER,
    TURRET,
    LOCATION_AMOUNT,
};

// Type of errors; subject to change
enum ErrorType
{
    IMU_DATA_NOT_INITIALIZED = 0,
    IMU_NOT_RECEIVING_PROPERLY,
    INVALID_MESSAGE_LENGTH,
    NULL_MOTOR_ID,
    CRC_FAILURE,
    MESSAGE_LENGTH_OVERFLOW,
    RUN_TIME_OVERFLOW,
    MOTOR_ID_OUT_OF_BOUNDS,
    ADDING_NULLPTR_COMMAND,
    ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES,
    ZERO_DESIRED_AGITATOR_ROTATE_TIME,
    INVALID_REMOVE,
    INVALID_KEY_MAP_TYPE,
    INVALID_ADD,
    MOTOR_OFFLINE,
    INVALID_MOTOR_OUTPUT,
    ERROR_TYPE_AMOUNT
};

class SystemError
{
public:
    SystemError();

    SystemError(
        const std::string& desc,
        int line,
        const std::string& file,
        Location l,
        ErrorType et);

    int getLineNumber() const;

    std::string getDescription() const;

    std::string getFilename() const;

    Location getLocation() const;

    ErrorType getErrorType() const;

private:
    int lineNumber;

    std::string description;

    std::string filename;

    Location location;

    ErrorType errorType;
};

}  // namespace errors

}  // namespace aruwlib

#endif
