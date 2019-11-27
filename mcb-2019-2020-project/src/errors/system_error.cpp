#include "src/errors/system_error.hpp"
#include "error_controller.hpp"

namespace aruwlib
{
namespace errors
{
    Location SystemError::getLocation() const
    {
        return this->location;
    }

    ErrorType SystemError::getErrorType() const
    {
        return this->errorType;
    }
}  // namespace errors

}  // namespace aruwlib
