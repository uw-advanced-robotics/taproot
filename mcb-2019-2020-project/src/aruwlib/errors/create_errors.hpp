#ifndef __CREATE_ERRORS_HPP__
#define __CREATE_ERRORS_HPP__

#include "aruwlib/Drivers.hpp"

#include "system_error.hpp"

namespace aruwlib
{
namespace errors
{
#define RAISE_ERROR(desc, l, et)                                                   \
    do                                                                             \
    {                                                                              \
        aruwlib::errors::SystemError stringError(desc, __LINE__, __FILE__, l, et); \
        Drivers::errorController.addToErrorList(stringError);                      \
    } while (0);

}  // namespace errors

}  // namespace aruwlib

#endif
