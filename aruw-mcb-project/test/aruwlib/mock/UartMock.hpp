/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef UART_MOCK_HPP_
#define UART_MOCK_HPP_

#include <aruwlib/communication/serial/uart.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class UartMock : public aruwlib::serial::Uart
{
public:
    MOCK_METHOD(bool, read, (aruwlib::serial::Uart::UartPort port, uint8_t *data), (override));
    MOCK_METHOD(
        std::size_t,
        read,
        (aruwlib::serial::Uart::UartPort port, uint8_t *data, std::size_t length),
        (override));
    MOCK_METHOD(
        std::size_t,
        discardReceiveBuffer,
        (aruwlib::serial::Uart::UartPort port),
        (override));
    MOCK_METHOD(bool, write, (aruwlib::serial::Uart::UartPort port, uint8_t data), (override));
    MOCK_METHOD(
        std::size_t,
        write,
        (aruwlib::serial::Uart::UartPort port, const uint8_t *data, std::size_t length),
        (override));
    MOCK_METHOD(bool, isWriteFinished, (aruwlib::serial::Uart::UartPort port), (const override));
    MOCK_METHOD(void, flushWriteBuffer, (aruwlib::serial::Uart::UartPort port), (override));
};  // class UartMock
}  // namespace mock
}  // namespace aruwlib

#endif  // UART_MOCK_HPP_
