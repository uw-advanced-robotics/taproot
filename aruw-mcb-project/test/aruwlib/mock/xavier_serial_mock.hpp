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

#ifndef XAVIER_SERIAL_MOCK_HPP_
#define XAVIER_SERIAL_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwlib/communication/serial/xavier_serial.hpp"

namespace aruwlib
{
namespace mock
{
class XavierSerialMock : public serial::XavierSerial
{
public:
    XavierSerialMock(Drivers *drivers);
    virtual ~XavierSerialMock();

    MOCK_METHOD(void, initializeCV, (), (override));
    MOCK_METHOD(void, messageReceiveCallback, (const SerialMessage &), (override));
    MOCK_METHOD(bool, sendMessage, (), (override));
    MOCK_METHOD(void, beginAutoAim, (), (override));
    MOCK_METHOD(void, stopAutoAim, (), (override));
    MOCK_METHOD(const TurretAimData &, getLastAimData, (), (const override));
    MOCK_METHOD(bool, lastAimDataValid, (), (const override));
    MOCK_METHOD(void, attachTurret, (control::turret::iTurretSubsystem *), (override));
    MOCK_METHOD(void, attachChassis, (control::chassis::iChassisSubsystem *), (override));
};  // class XavierSerialMock
}  // namespace mock
}  // namespace aruwlib

#endif  // XAVIER_SERIAL_MOCK_HPP_
