/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef CREATE_ERRORS_HPP_
#define CREATE_ERRORS_HPP_

#include "system_error.hpp"

namespace aruwlib
{
namespace errors
{
/**
 * Example for how to create and add an error. `drivers` is a pointer to an
 * `aruwlib::Drivers`, which contains an instance of an `ErrorController`.
 * 
 * Parameters l and et only exist for backwards compatibility with rm type a
 * board.
 *
 * @see ErrorController
 * @see SystemError
 *
 * ```cpp
 * RAISE_ERROR(
 *     drivers
 *     "Error in DJI Serial",
 *     aruwlib::errors::Location::DJI_SERIAL,
 *     aruwlib::errors::ErrorType::INVALID_CRC);
 * ```
 */
#define RAISE_ERROR(drivers, desc, l, et)                     \
    do                                                                      \
    {                                                                       \
        aruwlib::errors::SystemError stringError(desc, __LINE__, __FILE__); \
        drivers->errorController.addToErrorList(stringError);               \
    } while (0);

}  // namespace errors

}  // namespace aruwlib

#endif  // CREATE_ERRORS_HPP_
