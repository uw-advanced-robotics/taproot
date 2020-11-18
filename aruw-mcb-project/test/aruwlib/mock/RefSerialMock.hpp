/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef REF_SERIAL_MOCK_HPP_
#define REF_SERIAL_MOCK_HPP_

#include <aruwlib/communication/serial/ref_serial.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class RefSerialMock : public aruwlib::serial::RefSerial
{
public:
    RefSerialMock(aruwlib::Drivers* drivers) : aruwlib::serial::RefSerial(drivers) {}
    MOCK_METHOD(
        void,
        messageReceiveCallback,
        (const aruwlib::serial::DJISerial<true>::SerialMessage& completeMessage),
        (override));
    MOCK_METHOD(const aruwlib::serial::RefSerial::RobotData&, getRobotData, (), (const override));
    MOCK_METHOD(const aruwlib::serial::RefSerial::GameData&, getGameData, (), (const override));
    MOCK_METHOD(
        void,
        sendDisplayData,
        (const aruwlib::serial::RefSerial::DisplayData& displayData),
        (override));
};  // class RefSerialMock
}  // namespace mock
}  // namespace aruwlib

#endif  // REF_SERIAL_MOCK_HPP_
