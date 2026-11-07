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

#ifndef TAPROOT_ERROR_CONTROLLER_HPP_
#define TAPROOT_ERROR_CONTROLLER_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/util_macros.hpp"

#include "modm/container.hpp"

#include "system_error.hpp"

namespace tap
{
class Drivers;
}

namespace tap::errors
{
/**
 * The ErrorController stores the errors that are currently active.
 *
 * Use the `RAISE_ERROR` macro to add errors to the main ErrorController.
 */
class ErrorController
{
public:
    static constexpr std::size_t ERROR_LIST_MAX_SIZE = 16;
    using error_index_t = modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE>::Index;

    ErrorController(Drivers* drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(ErrorController)
    mockable ~ErrorController() = default;

    /**
     * Adds the passed in error to the ErrorController if no identical errors are already in
     * the ErrorController.
     *
     * @param[in] error The SystemError to add to the ErrorController.
     */
    mockable void addToErrorList(const SystemError& error);

    modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE> getErrorList() const { return errorList; }

    bool removeSystemErrorAtIndex(error_index_t index);

private:
    friend class ErrorControllerTester;
    friend class ErrorTerminalHandler;

    Drivers* drivers;

    modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE> errorList;

    void removeAllSystemErrors();

};  // class ErrorController
}  // namespace tap::errors

#endif  // TAPROOT_ERROR_CONTROLLER_HPP_
