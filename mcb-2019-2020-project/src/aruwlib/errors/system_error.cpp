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

#include "system_error.hpp"

namespace aruwlib
{
namespace errors
{
SystemError::SystemError()
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

SystemError::SystemError(const char *desc, int line, const char *file, Location l, ErrorType et)
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

int SystemError::getLineNumber() const { return lineNumber; }

const char *SystemError::getDescription() const { return description; }

const char *SystemError::getFilename() const { return filename; }

Location SystemError::getLocation() const { return location; }

ErrorType SystemError::getErrorType() const { return errorType; }

}  // namespace errors

}  // namespace aruwlib
