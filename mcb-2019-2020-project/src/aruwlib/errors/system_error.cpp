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
