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

#ifndef ERROR_CONTROLLER_HPP_
#define ERROR_CONTROLLER_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/terminal_serial.hpp"
#include "tap/util_macros.hpp"

#include "modm/container.hpp"

#include "system_error.hpp"

namespace tap
{
class Drivers;
namespace errors
{
/**
 * The ErrorController stores the errors that are currently active and displays errors
 * via the terminal serial.
 *
 * Use the `RAISE_ERROR` macro to add errors to the main ErrorController.
 */
class ErrorController : public tap::communication::serial::TerminalSerialCallbackInterface
{
public:
    static constexpr std::size_t ERROR_LIST_MAX_SIZE = 16;
    using error_index_t = modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE>::Index;

    /**
     * Constrcuts an ErrorController with a display time for each error specified
     * by `ERROR_ROTATE_TIME`.
     */
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

    void init();

    /**
     * @param[in] inputLine The user input to be processed.
     * @param[out] outputStream The stream to write information to.
     * @return `true` if the inputLine was valid and was parsed correctly, `false` otherwise.
     */
    bool terminalSerialCallback(char* inputLine, modm::IOStream& outputStream, bool) override;

    void terminalSerialStreamCallback(modm::IOStream&) override {}

private:
    friend class ErrorControllerTester;

    Drivers* drivers;

    modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE> errorList;

    bool removeSystemErrorAtIndex(error_index_t index);

    void removeAllSystemErrors();

    void displayAllErrors(modm::IOStream& outputStream);

    void removeTerminalError(int index, modm::IOStream& outputStream);

    void clearAllTerminalErrors(modm::IOStream& outputStream);

    void help(modm::IOStream& outputStream);
};  // class ErrorController
}  // namespace errors
}  // namespace tap

#endif  // ERROR_CONTROLLER_HPP_
