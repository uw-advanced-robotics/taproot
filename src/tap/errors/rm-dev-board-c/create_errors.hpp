/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef CREATE_ERRORS_HPP_
#define CREATE_ERRORS_HPP_

#include "system_error.hpp"

namespace tap
{
namespace errors
{
/**
 * Example for how to create and add an error. `drivers` is a pointer to an
 * `taproot::Drivers`, which contains an instance of an `ErrorController`.
 *
 * Parameters l and et only exist for backwards compatibility with rm type a
 * board.
 *
 * @see ErrorController
 * @see SystemError
 *
 * ```cpp
 * RAISE_ERROR(drivers, "Error in DJI Serial");
 * ```
 */
#define RAISE_ERROR(drivers, desc, l, et)                               \
    do                                                                  \
    {                                                                   \
        tap::errors::SystemError stringError(desc, __LINE__, __FILE__); \
        drivers->errorController.addToErrorList(stringError);           \
    } while (0);

}  // namespace errors

}  // namespace tap

#endif  // CREATE_ERRORS_HPP_
