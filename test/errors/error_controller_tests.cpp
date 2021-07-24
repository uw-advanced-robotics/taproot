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
    ErrorControllerTester(aruwlib::Drivers *drivers) : errorController(drivers) {}
    ErrorController errorController;
    bool removeSystemError(uint index) { return errorController.removeSystemErrorAtIndex(index); }
    void setCurrDisplayIndex(uint index) { errorController.currentDisplayIndex = index; }
    uint getErrorListSize() { return errorController.errorList.getSize(); }
    uint getCurrentDisplayIndex() { return errorController.currentDisplayIndex; }
};
}  // namespace tap::errors

using aruwlib::Drivers;
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
    SystemError se("error", __LINE__, __FILE__, Location::CAN_RX, 1);

    ec.errorController.addToErrorList(se);

    EXPECT_EQ(1, ec.getErrorListSize());
    EXPECT_FALSE(ec.removeSystemError(1));
    EXPECT_EQ(1, ec.getErrorListSize());
    EXPECT_TRUE(ec.removeSystemError(0));
    EXPECT_EQ(0, ec.getErrorListSize());
    EXPECT_FALSE(ec.removeSystemError(0));
}

TEST(ErrorController, removeSystemError_multiple_errors_successfully_removed_from_end)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    SystemError se1("error", __LINE__, __FILE__, Location::CAN_RX, 1);
    SystemError se2("error 2", __LINE__, __FILE__, Location::COMMAND_SCHEDULER, 1);
    SystemError se3("error 3", __LINE__, __FILE__, Location::DJI_SERIAL, 1);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    // first try currentDisplayIndex at end
    ec.setCurrDisplayIndex(2);
    EXPECT_TRUE(ec.removeSystemError(2));
    EXPECT_EQ(0, ec.getCurrentDisplayIndex());
    EXPECT_EQ(2, ec.getErrorListSize());

    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    // next try with currentDisplayIndex before end
    ec.setCurrDisplayIndex(1);
    EXPECT_TRUE(ec.removeSystemError(2));
    EXPECT_EQ(1, ec.getCurrentDisplayIndex());
    EXPECT_EQ(2, ec.getErrorListSize());

    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    // finally try with currentDisplayIndex at start
    ec.setCurrDisplayIndex(0);
    EXPECT_TRUE(ec.removeSystemError(2));
    EXPECT_EQ(0, ec.getCurrentDisplayIndex());
    EXPECT_EQ(2, ec.getErrorListSize());
}

TEST(ErrorController, removeSystemError_at_start_usually_shifts_currentDisplayIndex_left)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    SystemError se1("error", __LINE__, __FILE__, Location::CAN_RX, 1);
    SystemError se2("error 2", __LINE__, __FILE__, Location::COMMAND_SCHEDULER, 1);
    SystemError se3("error 3", __LINE__, __FILE__, Location::DJI_SERIAL, 1);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    // first try currentDisplayIndex at end
    ec.setCurrDisplayIndex(2);
    EXPECT_TRUE(ec.removeSystemError(0));
    EXPECT_EQ(1, ec.getCurrentDisplayIndex());
    EXPECT_EQ(2, ec.getErrorListSize());

    ec.errorController.addToErrorList(se1);
    EXPECT_EQ(3, ec.getErrorListSize());

    // next try with currentDisplayIndex before end
    ec.setCurrDisplayIndex(1);
    EXPECT_TRUE(ec.removeSystemError(0));
    EXPECT_EQ(0, ec.getCurrentDisplayIndex());
    EXPECT_EQ(2, ec.getErrorListSize());

    ec.errorController.addToErrorList(se2);
    EXPECT_EQ(3, ec.getErrorListSize());

    // finally try with currentDisplayIndex at start
    ec.setCurrDisplayIndex(0);
    EXPECT_TRUE(ec.removeSystemError(0));
    EXPECT_EQ(0, ec.getCurrentDisplayIndex());
    EXPECT_EQ(2, ec.getErrorListSize());
}

TEST(
    ErrorController,
    removeSystemError_from_middle_shifts_currentDisplayIndex_left_if_removed_index_smaller)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    SystemError se1("error", __LINE__, __FILE__, Location::CAN_RX, 1);
    SystemError se2("error 2", __LINE__, __FILE__, Location::COMMAND_SCHEDULER, 1);
    SystemError se3("error 3", __LINE__, __FILE__, Location::DJI_SERIAL, 1);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    // first try currentDisplayIndex at end
    ec.setCurrDisplayIndex(2);
    EXPECT_TRUE(ec.removeSystemError(1));
    EXPECT_EQ(1, ec.getCurrentDisplayIndex());
    EXPECT_EQ(2, ec.getErrorListSize());

    ec.errorController.addToErrorList(se2);
    EXPECT_EQ(3, ec.getErrorListSize());

    // next try with currentDisplayIndex before end
    ec.setCurrDisplayIndex(1);
    EXPECT_TRUE(ec.removeSystemError(1));
    EXPECT_EQ(1, ec.getCurrentDisplayIndex());
    EXPECT_EQ(2, ec.getErrorListSize());

    ec.errorController.addToErrorList(se3);
    EXPECT_EQ(3, ec.getErrorListSize());

    // finally try with currentDisplayIndex at start
    ec.setCurrDisplayIndex(0);
    EXPECT_TRUE(ec.removeSystemError(1));
    EXPECT_EQ(0, ec.getCurrentDisplayIndex());
    EXPECT_EQ(2, ec.getErrorListSize());
}

TEST(ErrorController, removeSystemError_index_before_currentDisplayIndex_shifts_index_left)
{
    Drivers drivers;
    ErrorControllerTester ec(&drivers);
    SystemError se1("error", __LINE__, __FILE__, Location::CAN_RX, 1);
    SystemError se2("error 2", __LINE__, __FILE__, Location::COMMAND_SCHEDULER, 1);
    SystemError se3("error 3", __LINE__, __FILE__, Location::DJI_SERIAL, 1);

    ec.errorController.addToErrorList(se1);
    ec.errorController.addToErrorList(se2);
    ec.errorController.addToErrorList(se3);

    // display index 2, remove index 0
    ec.setCurrDisplayIndex(2);
    ec.removeSystemError(0);
    EXPECT_EQ(1, ec.getCurrentDisplayIndex());

    ec.errorController.addToErrorList(se1);
    EXPECT_EQ(3, ec.getErrorListSize());

    // display index 1, remove index 0
    ec.setCurrDisplayIndex(1);
    ec.removeSystemError(0);
    EXPECT_EQ(0, ec.getCurrentDisplayIndex());

    ec.errorController.addToErrorList(se2);
    EXPECT_EQ(3, ec.getErrorListSize());

    // display index 2, remove index 1
    ec.setCurrDisplayIndex(2);
    ec.removeSystemError(1);
    EXPECT_EQ(1, ec.getCurrentDisplayIndex());
}
