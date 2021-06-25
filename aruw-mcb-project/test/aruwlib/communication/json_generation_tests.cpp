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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "aruwlib/communication/can/can.hpp"
#include "aruwlib/communication/tcp-server/json_messages.hpp"
#include "aruwlib/drivers.hpp"

#include "../mock/dji_motor_mock.hpp"

using aruwlib::Drivers;
using aruwlib::mock::DjiMotorMock;
using namespace aruwlib::can;
using namespace aruwlib::motor;
using namespace aruwlib::communication::json;
using namespace testing;

TEST(JSONMessages, ReturnsCorrectString)
{
    Drivers drivers;
    DjiMotorMock mockMotor(&drivers, MotorId::MOTOR1, CanBus::CAN_BUS1, false, "MockMotor");
    EXPECT_CALL(mockMotor, getCanBus()).Times(1).WillOnce(Return(aruwlib::can::CanBus::CAN_BUS2));
    EXPECT_CALL(mockMotor, getMotorIdentifier()).Times(1);
    EXPECT_CALL(mockMotor, getShaftRPM()).Times(1);
    EXPECT_CALL(mockMotor, getTorque()).Times(1);
    EXPECT_CALL(mockMotor, getEncoderUnwrapped()).Times(1);
    EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler).Times(AnyNumber());

    std::string s = makeMotorMessage(mockMotor);

    EXPECT_EQ(
        "{\"messageType\":\"motor\",\"canBus\":2,\"motorID\":0,"
        "\"shaftRPM\":0,\"torque\":0,\"encoderValue\":0}",
        s);
}
