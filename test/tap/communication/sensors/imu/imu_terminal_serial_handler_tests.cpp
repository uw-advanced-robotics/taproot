/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <gtest/gtest.h>

#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/imu_interface_mock.hpp"
#include "tap/stub/terminal_device_stub.hpp"

using namespace tap;
using namespace tap::mock;
using namespace tap::communication::sensors::imu;
using namespace testing;

class ImuTerminalSerialHandlerTest : public Test
{
protected:
    ImuTerminalSerialHandlerTest()
        : serialHandler(&drivers, &imu),
          terminalDevice(&drivers),
          stream(terminalDevice)
    {
    }

    void SetUp() override { ON_CALL(imu, getName).WillByDefault(Return("imu")); }

    Drivers drivers;
    NiceMock<ImuInterfaceMock> imu;
    ImuTerminalSerialHandler serialHandler;
    tap::stub::TerminalDeviceStub terminalDevice;
    modm::IOStream stream;
};

TEST_F(ImuTerminalSerialHandlerTest, init__adds_self_to_terminal_serial)
{
    EXPECT_CALL(drivers.terminalSerial, addHeader(_, &serialHandler));

    serialHandler.init();
}

TEST_F(ImuTerminalSerialHandlerTest, terminalSerialCallback__h_prints_usage)
{
    char input[] = "-h";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST_F(ImuTerminalSerialHandlerTest, terminalSerialCallback__empty_string_input_returns_false)
{
    char input[] = "";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST_F(ImuTerminalSerialHandlerTest, terminalSerialCallback__no_args_returns_false)
{
    char input[] = "  \t\n    \r ";

    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST_F(ImuTerminalSerialHandlerTest, terminalSerialCallback__incorrect_args_returns_false)
{
    char input[] = "anglfes gysros tempehratre";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST_F(ImuTerminalSerialHandlerTest, terminalSerialCallback__angle_input_return_angle_values)
{
    ON_CALL(imu, getPitch).WillByDefault(Return(12.34));
    ON_CALL(imu, getRoll).WillByDefault(Return(90.54));
    ON_CALL(imu, getYaw).WillByDefault(Return(180.43));

    char input[] = "angle";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("pit"));
    EXPECT_THAT(output, HasSubstr("rol"));
    EXPECT_THAT(output, HasSubstr("yaw"));
    EXPECT_THAT(output, HasSubstr("12.34"));
    EXPECT_THAT(output, HasSubstr("90.54"));
    EXPECT_THAT(output, HasSubstr("180.43"));
}

TEST_F(ImuTerminalSerialHandlerTest, terminalSerialCallback__gyro_input_return_gyro_values)
{
    ON_CALL(imu, getGx).WillByDefault(Return(12.34));
    ON_CALL(imu, getGy).WillByDefault(Return(90.54));
    ON_CALL(imu, getGz).WillByDefault(Return(180.43));

    char input[] = "gyro";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("gx"));
    EXPECT_THAT(output, HasSubstr("gy"));
    EXPECT_THAT(output, HasSubstr("gz"));
    EXPECT_THAT(output, HasSubstr("12.34"));
    EXPECT_THAT(output, HasSubstr("90.54"));
    EXPECT_THAT(output, HasSubstr("180.43"));
}

TEST_F(ImuTerminalSerialHandlerTest, terminalSerialCallback__accel_input_return_accel_value)
{
    ON_CALL(imu, getAx).WillByDefault(Return(12.34));
    ON_CALL(imu, getAy).WillByDefault(Return(90.54));
    ON_CALL(imu, getAz).WillByDefault(Return(180.43));

    char input[] = "accel";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("ax"));
    EXPECT_THAT(output, HasSubstr("ay"));
    EXPECT_THAT(output, HasSubstr("az"));
    EXPECT_THAT(output, HasSubstr("12.34"));
    EXPECT_THAT(output, HasSubstr("90.54"));
    EXPECT_THAT(output, HasSubstr("180.43"));
}

TEST_F(ImuTerminalSerialHandlerTest, terminalSerialCallback__temp_input_return_temp_value)
{
    ON_CALL(imu, getTemp).WillByDefault(Return(32.42));

    char input[] = "temp";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("temp"));
    EXPECT_THAT(output, HasSubstr("32.42"));
}

TEST_F(
    ImuTerminalSerialHandlerTest,
    terminalSerialCallback__temp_gyro_angle_accel_return_all_values)
{
    ON_CALL(imu, getPitch).WillByDefault(Return(123.54));
    ON_CALL(imu, getRoll).WillByDefault(Return(-42.05));
    ON_CALL(imu, getYaw).WillByDefault(Return(1.89));
    ON_CALL(imu, getGx).WillByDefault(Return(12.34));
    ON_CALL(imu, getGy).WillByDefault(Return(-12.53));
    ON_CALL(imu, getGz).WillByDefault(Return(-54.01));
    ON_CALL(imu, getAx).WillByDefault(Return(12.34));
    ON_CALL(imu, getAy).WillByDefault(Return(90.54));
    ON_CALL(imu, getAz).WillByDefault(Return(180.43));
    ON_CALL(imu, getTemp).WillByDefault(Return(32.42));

    char input[] = "temp gyro angle accel";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();

    EXPECT_THAT(output, HasSubstr("pit"));
    EXPECT_THAT(output, HasSubstr("rol"));
    EXPECT_THAT(output, HasSubstr("yaw"));
    EXPECT_THAT(output, HasSubstr("gx"));
    EXPECT_THAT(output, HasSubstr("gy"));
    EXPECT_THAT(output, HasSubstr("gz"));
    EXPECT_THAT(output, HasSubstr("ax"));
    EXPECT_THAT(output, HasSubstr("ay"));
    EXPECT_THAT(output, HasSubstr("az"));
    EXPECT_THAT(output, HasSubstr("temp"));

    EXPECT_THAT(output, HasSubstr("123.54"));
    EXPECT_THAT(output, HasSubstr("-42.05"));
    EXPECT_THAT(output, HasSubstr("1.89"));
    EXPECT_THAT(output, HasSubstr("12.34"));
    EXPECT_THAT(output, HasSubstr("-12.53"));
    EXPECT_THAT(output, HasSubstr("-54.01"));
    EXPECT_THAT(output, HasSubstr("12.34"));
    EXPECT_THAT(output, HasSubstr("90.54"));
    EXPECT_THAT(output, HasSubstr("180.43"));
    EXPECT_THAT(output, HasSubstr("32.42"));
}

TEST_F(ImuTerminalSerialHandlerTest, terminalSerialCallback__temp_with_bad_input_return_false)
{
    char input[] = "temp ldskf";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST_F(
    ImuTerminalSerialHandlerTest,
    terminalSerialCallback__streaming_mode_with_temp_displays_temp_over_and_over)
{
    ON_CALL(imu, getTemp).WillByDefault(Return(32.42));

    char input[] = "temp";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, true));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();

    EXPECT_THAT(output, HasSubstr("temp"));
    EXPECT_THAT(output, HasSubstr("32.42"));

    ON_CALL(imu, getTemp).WillByDefault(Return(12.31));

    serialHandler.terminalSerialStreamCallback(stream);

    output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, Not(HasSubstr("temp")));
    EXPECT_THAT(output, HasSubstr("12.31"));

    ON_CALL(imu, getTemp).WillByDefault(Return(54.42));

    serialHandler.terminalSerialStreamCallback(stream);

    output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, Not(HasSubstr("temp")));
    EXPECT_THAT(output, HasSubstr("54.42"));
}
