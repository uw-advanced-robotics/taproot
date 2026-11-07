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

#include <gmock/gmock.h>

#include "tap/drivers.hpp"
#include "tap/errors/error_controller.hpp"
#include "tap/errors/error_terminal_handler.hpp"
#include "tap/stub/terminal_device_stub.hpp"

namespace tap::errors
{
class ErrorTerminalHandlerTester
{
public:
    ErrorTerminalHandlerTester(tap::Drivers* drivers)
        : errorController(drivers),
          errorTerminalHandler(errorController)
    {
    }
    void displayAllErrors(modm::IOStream& outputStream)
    {
        errorTerminalHandler.displayAllErrors(outputStream);
    }

    ErrorController errorController;
    ErrorTerminalHandler errorTerminalHandler;
};
}  // namespace tap::errors

using tap::Drivers;
using namespace tap::errors;
using namespace testing;

TEST(ErrorTerminalHandler, displayAllErrors__with_no_errors_displays_no_errors)
{
    Drivers drivers;
    ErrorTerminalHandlerTester ec(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    ec.displayAllErrors(stream);

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    std::transform(output.begin(), output.end(), output.begin(), ::tolower);

    EXPECT_THAT(output, HasSubstr("no errors found"));
}

TEST(ErrorTerminalHandler, displayAllErrors__contains_error_descriptions_of_all_errors)
{
    Drivers drivers;
    ErrorTerminalHandlerTester ec(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    SystemError se1("error1", __LINE__, __FILE__);
    SystemError se2("error2", __LINE__, __FILE__);
    SystemError se3("error3", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);

    ec.displayAllErrors(stream);

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    std::transform(output.begin(), output.end(), output.begin(), ::tolower);

    EXPECT_THAT(output, Not(HasSubstr("no errors found")));

    EXPECT_THAT(output, HasSubstr("error1"));
    EXPECT_THAT(output, HasSubstr("error2"));
    EXPECT_THAT(output, HasSubstr("error3"));
}

TEST(ErrorTerminalHandler, terminalSerialCallback__streamingEnabled_true_function_does_nothing)
{
    Drivers drivers;
    ErrorTerminalHandlerTester ec(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char help[] = " -H";
    EXPECT_FALSE(ec.errorTerminalHandler.terminalSerialCallback(help, stream, true));
}

TEST(ErrorTerminalHandler, terminalSerialCallback__help_or_nothing_returns_help_string)
{
    Drivers drivers;
    ErrorTerminalHandlerTester ec(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char help[] = "-H";
    EXPECT_TRUE(ec.errorTerminalHandler.terminalSerialCallback(help, stream, false));
    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));

    char error[] = "  ";
    EXPECT_FALSE(ec.errorTerminalHandler.terminalSerialCallback(error, stream, false));
    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(ErrorTerminalHandler, terminalSerialCallback__printall_prints_all_errors)
{
    Drivers drivers;
    ErrorTerminalHandlerTester ec(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    SystemError se1("error1", __LINE__, __FILE__);
    SystemError se2("error2", __LINE__, __FILE__);
    SystemError se3("error3", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);

    char printAll[] = "printall";
    EXPECT_TRUE(ec.errorTerminalHandler.terminalSerialCallback(printAll, stream, false));
    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();

    EXPECT_THAT(output, HasSubstr("error1"));
    EXPECT_THAT(output, HasSubstr("error2"));
    EXPECT_THAT(output, HasSubstr("error3"));
}

TEST(ErrorTerminalHandler, terminalSerialCallback__remove_at_index_removes_correct_error)
{
    Drivers drivers;
    ErrorTerminalHandlerTester ec(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    SystemError se1("error1", __LINE__, __FILE__);
    SystemError se2("error2", __LINE__, __FILE__);
    SystemError se3("error3", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);

    char remove[] = "remove 1";
    EXPECT_TRUE(ec.errorTerminalHandler.terminalSerialCallback(remove, stream, false));
    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    std::transform(output.begin(), output.end(), output.begin(), ::tolower);

    EXPECT_THAT(output, HasSubstr("removing"));

    // call printall to check which errors still remain
    char printAll[] = "printall";
    EXPECT_TRUE(ec.errorTerminalHandler.terminalSerialCallback(printAll, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), Not(HasSubstr("error2")));
    EXPECT_EQ(2, ec.errorController.getErrorList().getSize());
}

TEST(ErrorTerminalHandler, terminalSerialCallback__remove_at_index_doesnot_remove_invalid_index)
{
    Drivers drivers;
    ErrorTerminalHandlerTester ec(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    SystemError se1("error1", __LINE__, __FILE__);
    SystemError se2("error2", __LINE__, __FILE__);
    SystemError se3("error3", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);

    char remove[] = "remove -1";
    EXPECT_TRUE(ec.errorTerminalHandler.terminalSerialCallback(remove, stream, false));
    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    std::transform(output.begin(), output.end(), output.begin(), ::tolower);

    EXPECT_THAT(output, HasSubstr("invalid index"));

    char remove2[] = "remove 3";
    EXPECT_TRUE(ec.errorTerminalHandler.terminalSerialCallback(remove2, stream, false));
    output = terminalDevice.readAllItemsFromWriteBufferToString();
    std::transform(output.begin(), output.end(), output.begin(), ::tolower);

    EXPECT_THAT(output, HasSubstr("invalid index"));
}

TEST(ErrorTerminalHandler, terminalSerialCallback__removeall_removes_all_errors)
{
    Drivers drivers;
    ErrorTerminalHandlerTester ec(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    SystemError se1("error1", __LINE__, __FILE__);
    SystemError se2("error2", __LINE__, __FILE__);
    SystemError se3("error3", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);

    char remove[] = "removeall";
    EXPECT_TRUE(ec.errorTerminalHandler.terminalSerialCallback(remove, stream, false));
    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    std::transform(output.begin(), output.end(), output.begin(), ::tolower);

    EXPECT_THAT(output, HasSubstr("removing"));

    // call printall to check which errors still remain
    char printAll[] = "printall";
    EXPECT_TRUE(ec.errorTerminalHandler.terminalSerialCallback(printAll, stream, false));
    output = terminalDevice.readAllItemsFromWriteBufferToString();

    EXPECT_THAT(output, Not(HasSubstr("error1")));
    EXPECT_THAT(output, Not(HasSubstr("error2")));
    EXPECT_THAT(output, Not(HasSubstr("error3")));
    EXPECT_EQ(0, ec.errorController.getErrorList().getSize());
}
