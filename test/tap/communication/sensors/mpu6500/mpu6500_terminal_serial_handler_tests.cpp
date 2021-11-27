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

#include <gtest/gtest.h>

#include "tap/communication/sensors/mpu6500/mpu6500_terminal_serial_handler.hpp"
#include "tap/drivers.hpp"
#include "tap/stub/terminal_device_stub.hpp"

using namespace tap;
using namespace tap::sensors;
using namespace testing;

TEST(Mpu6500TerminalSerialHandler, init__adds_self_to_terminal_serial)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);

    EXPECT_CALL(drivers.terminalSerial, addHeader(_, &serialHandler));

    serialHandler.init();
}

TEST(Mpu6500TerminalSerialHandler, terminalSerialCallback__no_args_returns_false)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "  \t\n    \r ";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(Mpu6500TerminalSerialHandler, terminalSerialCallback__incorrect_args_returns_false)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "anglfes gysros tempehratre";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(Mpu6500TerminalSerialHandler, terminalSerialCallback__angle_input_return_angle_values)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    ON_CALL(drivers.mpu6500, getPitch).WillByDefault(Return(12.34));
    ON_CALL(drivers.mpu6500, getRoll).WillByDefault(Return(90.54));
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(Return(180.43));

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

TEST(Mpu6500TerminalSerialHandler, terminalSerialCallback__gyro_input_return_gyro_values)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    ON_CALL(drivers.mpu6500, getGx).WillByDefault(Return(12.34));
    ON_CALL(drivers.mpu6500, getGy).WillByDefault(Return(90.54));
    ON_CALL(drivers.mpu6500, getGz).WillByDefault(Return(180.43));

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

TEST(Mpu6500TerminalSerialHandler, terminalSerialCallback__accel_input_return_accel_value)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    ON_CALL(drivers.mpu6500, getAx).WillByDefault(Return(12.34));
    ON_CALL(drivers.mpu6500, getAy).WillByDefault(Return(90.54));
    ON_CALL(drivers.mpu6500, getAz).WillByDefault(Return(180.43));

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

TEST(Mpu6500TerminalSerialHandler, terminalSerialCallback__temp_input_return_temp_value)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    ON_CALL(drivers.mpu6500, getTemp).WillByDefault(Return(32.42));

    char input[] = "temp";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("temp"));
    EXPECT_THAT(output, HasSubstr("32.42"));
}

TEST(Mpu6500TerminalSerialHandler, terminalSerialCallback__temp_gyro_angle_accel_return_all_values)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    ON_CALL(drivers.mpu6500, getPitch).WillByDefault(Return(123.54));
    ON_CALL(drivers.mpu6500, getRoll).WillByDefault(Return(-42.05));
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(Return(1.89));
    ON_CALL(drivers.mpu6500, getGx).WillByDefault(Return(12.34));
    ON_CALL(drivers.mpu6500, getGy).WillByDefault(Return(-12.53));
    ON_CALL(drivers.mpu6500, getGz).WillByDefault(Return(-54.01));
    ON_CALL(drivers.mpu6500, getAx).WillByDefault(Return(12.34));
    ON_CALL(drivers.mpu6500, getAy).WillByDefault(Return(90.54));
    ON_CALL(drivers.mpu6500, getAz).WillByDefault(Return(180.43));
    ON_CALL(drivers.mpu6500, getTemp).WillByDefault(Return(32.42));

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

TEST(Mpu6500TerminalSerialHandler, terminalSerialCallback__temp_with_bad_input_return_false)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "temp ldskf";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(
    Mpu6500TerminalSerialHandler,
    terminalSerialCallback__streaming_mode_with_temp_displays_temp_over_and_over)
{
    Drivers drivers;
    Mpu6500TerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    ON_CALL(drivers.mpu6500, getTemp).WillByDefault(Return(32.42));

    char input[] = "temp";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, true));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();

    EXPECT_THAT(output, HasSubstr("temp"));
    EXPECT_THAT(output, HasSubstr("32.42"));

    ON_CALL(drivers.mpu6500, getTemp).WillByDefault(Return(12.31));

    serialHandler.terminalSerialStreamCallback(stream);

    output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, Not(HasSubstr("temp")));
    EXPECT_THAT(output, HasSubstr("12.31"));

    ON_CALL(drivers.mpu6500, getTemp).WillByDefault(Return(54.42));

    serialHandler.terminalSerialStreamCallback(stream);

    output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, Not(HasSubstr("temp")));
    EXPECT_THAT(output, HasSubstr("54.42"));
}
