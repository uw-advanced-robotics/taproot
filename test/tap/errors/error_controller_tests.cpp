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

namespace tap::errors
{
class ErrorControllerTester
{
public:
    ErrorControllerTester(tap::Drivers* drivers) : errorController(drivers) {}
    bool removeSystemError(uint index) { return errorController.removeSystemErrorAtIndex(index); }
    uint getErrorListSize() { return errorController.errorList.getSize(); }
    bool removeSystemErrorAtIndex(ErrorController::error_index_t index)
    {
        return errorController.removeSystemErrorAtIndex(index);
    }
    void removeAllSystemErrors() { errorController.removeAllSystemErrors(); }
    void displayAllErrors(modm::IOStream& outputStream)
    {
        errorController.displayAllErrors(outputStream);
    }

    ErrorController errorController;
};
}  // namespace tap::errors

using tap::Drivers;
using namespace tap::errors;

TEST(ErrorController, removeSystemError_no_errors_in_error_controller_doesnot_segfault)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    EXPECT_FALSE(ec.removeSystemError(0));
    EXPECT_FALSE(ec.removeSystemError(1));
    EXPECT_FALSE(ec.removeSystemError(2));
}

TEST(ErrorController, removeSystemError_single_error_removed_at_zero_index)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    SystemError se("error", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se);

    EXPECT_EQ(1, ec.getErrorListSize());
    EXPECT_FALSE(ec.removeSystemError(1));
    EXPECT_EQ(1, ec.getErrorListSize());
    EXPECT_TRUE(ec.removeSystemError(0));
    EXPECT_EQ(0, ec.getErrorListSize());
    EXPECT_FALSE(ec.removeSystemError(0));
}

TEST(ErrorController, removeSystemError_from_end)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    SystemError se1("error", __LINE__, __FILE__);
    SystemError se2("error 2", __LINE__, __FILE__);
    SystemError se3("error 3", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    EXPECT_TRUE(ec.removeSystemError(2));
    EXPECT_EQ(2, ec.getErrorListSize());
    EXPECT_TRUE(ec.removeSystemError(1));
    EXPECT_EQ(1, ec.getErrorListSize());
    EXPECT_TRUE(ec.removeSystemError(0));
    EXPECT_EQ(0, ec.getErrorListSize());
}

TEST(ErrorController, removeSystemError_from_start)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    SystemError se1("error", __LINE__, __FILE__);
    SystemError se2("error 2", __LINE__, __FILE__);
    SystemError se3("error 3", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    EXPECT_TRUE(ec.removeSystemError(0));
    EXPECT_EQ(2, ec.getErrorListSize());
    EXPECT_TRUE(ec.removeSystemError(0));
    EXPECT_EQ(1, ec.getErrorListSize());
    EXPECT_TRUE(ec.removeSystemError(0));
    EXPECT_EQ(0, ec.getErrorListSize());
}

TEST(ErrorController, removeSystemError_from_middle)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    SystemError se1("error", __LINE__, __FILE__);
    SystemError se2("error 2", __LINE__, __FILE__);
    SystemError se3("error 3", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    // first try currentDisplayIndex at end
    EXPECT_TRUE(ec.removeSystemError(1));
    EXPECT_EQ(2, ec.getErrorListSize());
}

TEST(ErrorController, removeAllSystemErrors__when_no_errors_does_nothing_but_doesnt_segfault)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);

    ec.removeAllSystemErrors();
}

TEST(ErrorController, removeAllSystemErrors__multiple_errors_removed)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    SystemError se1("error", __LINE__, __FILE__);
    SystemError se2("error 2", __LINE__, __FILE__);
    SystemError se3("error 3", __LINE__, __FILE__);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    ec.removeAllSystemErrors();
    EXPECT_EQ(0, ec.getErrorListSize());
}

// TODO the rest

TEST(ErrorController, displayAllErrors__with_no_errors_displays_no_errors) {}

TEST(ErrorController, displayAllErrors__contains_error_descriptions_of_all_errors) {}

TEST(ErrorController, terminalSerialCallback__) {}