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

#ifndef XAVIER_SERIAL_MOCK_HPP_
#define XAVIER_SERIAL_MOCK_HPP_

#include <aruwlib/communication/serial/xavier_serial.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class XavierSerialMock : public aruwlib::serial::XavierSerial
{
public:
    XavierSerialMock(aruwlib::Drivers* drivers) : aruwlib::serial::XavierSerial(drivers) {}
    MOCK_METHOD(void, initializeCV, (), (override));
    MOCK_METHOD(
        void,
        messageReceiveCallback,
        (const aruwlib::serial::DJISerial::SerialMessage& completeMessage),
        (override));
    MOCK_METHOD(
        void,
        sendMessage,
        (const aruwlib::serial::XavierSerial::IMUData& imuData,
         const aruwlib::serial::XavierSerial::ChassisData& chassisData,
         const aruwlib::serial::XavierSerial::TurretAimData& turretData,
         uint8_t robotId),
        (override));
    MOCK_METHOD(void, beginTargetTracking, (), (override));
    MOCK_METHOD(void, stopTargetTracking, (), (override));
    MOCK_METHOD(
        bool,
        getLastAimData,
        (aruwlib::serial::XavierSerial::TurretAimData * aimData),
        (const override));
};  // class XavierSerialMock
}  // namespace mock
}  // namespace aruwlib

#endif  // XAVIER_SERIAL_MOCK_HPP_
