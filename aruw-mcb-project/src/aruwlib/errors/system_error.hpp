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

#ifndef SYSTEM_ERROR_HPP_
#define SYSTEM_ERROR_HPP_

#include <string>

namespace aruwlib
{
namespace errors
{
/// Location of errors; subject to change
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
    SERVO,
    LOCATION_AMOUNT,
};

/// Type of errors; subject to change
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
    static const uint8_t ERROR_LOCATION_SIZE = 4;

    static const uint8_t ERROR_TYPE_SIZE = 8 - ERROR_LOCATION_SIZE;

    constexpr SystemError()
        : lineNumber(0),
          description("default"),
          filename("none"),
          location(LOCATION_AMOUNT),
          errorType(ERROR_TYPE_AMOUNT)
    {
        static_assert(
            LOCATION_AMOUNT <= ERROR_LOCATION_SIZE * ERROR_LOCATION_SIZE,
            "You have declared too many locations!");
        static_assert(
            ERROR_TYPE_AMOUNT <= ERROR_TYPE_SIZE * ERROR_TYPE_SIZE,
            "You have declared too many error types!");
    }

    constexpr SystemError(const char *desc, int line, const char *file, Location l, ErrorType et)
        : lineNumber(line),
          description(desc),
          filename(file),
          location(l),
          errorType(et)
    {
        static_assert(
            LOCATION_AMOUNT <= ERROR_LOCATION_SIZE * ERROR_LOCATION_SIZE,
            "You have declared too many locations!");
        static_assert(
            ERROR_TYPE_AMOUNT <= ERROR_TYPE_SIZE * ERROR_TYPE_SIZE,
            "You have declared too many error types!");
    }

    constexpr int getLineNumber() const { return lineNumber; }

    const char *getDescription() const { return description; }

    const char *getFilename() const { return filename; }

    constexpr Location getLocation() const { return location; }

    constexpr ErrorType getErrorType() const { return errorType; }

private:
    int lineNumber;

    const char *description;

    const char *filename;

    Location location;

    ErrorType errorType;
};  // class SystemError
}  // namespace errors
}  // namespace aruwlib

#endif  // SYSTEM_ERROR_HPP_
