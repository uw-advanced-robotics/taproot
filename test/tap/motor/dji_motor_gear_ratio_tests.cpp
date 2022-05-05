/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include <bitset>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/architecture/endianness_wrappers.hpp"

#include "modm/math/geometry/angle.hpp"

using namespace tap::motor;
using namespace testing;
using namespace tap;

static void setMotorEncoderValue(DjiMotor& motor, uint16_t encoderValue)
{
    uint8_t inLength(8);

    uint64_t data = 0;

    // manually set the proper bits to the encoder value
    *((uint8_t *)&data + 6) = *((uint8_t *)&encoderValue);
    *((uint8_t *)&data + 7) = *((uint8_t *)&encoderValue + 1);

    modm::can::Message message{motor.getMotorIdentifier(), inLength, data};

    // segfault is happening somewhere within this call
    motor.processMessage(message);
}

static void testMotorEncoderValue(
    float gearRatio,
    uint16_t encoderValue,
    float unwrappedAngleExpected)
{
    Drivers drivers;
    DjiMotor motor(
        &drivers,
        MotorId::MOTOR1,
        tap::can::CanBus::CAN_BUS1,
        false,
        "test motor",
        gearRatio);

    setMotorEncoderValue(motor, encoderValue);
    EXPECT_EQ(motor.getShaftAngleUnwrapped(), unwrappedAngleExpected);
    EXPECT_EQ(motor.getShaftAngleWrapped(), modm::Angle::normalize(unwrappedAngleExpected));
}

TEST(DjiMotor, getShaftAngleWrappedUnwrapped_M2006_zero)
{
    testMotorEncoderValue(DjiMotor::GEAR_RATIO_M2006, 0, 0.0f);
}

TEST(DjiMotor, getShaftAngleWrappedUnwrapped_M2006_half_rotation)
{
    testMotorEncoderValue(
        DjiMotor::GEAR_RATIO_M2006,
        DjiMotor::ENC_RESOLUTION / 2,
        0.5f * M_PI / DjiMotor::GEAR_RATIO_M2006);
}

TEST(DjiMotor, getShaftAngleWrappedUnwrapped_M2006_full_rotation)
{
    testMotorEncoderValue(
        DjiMotor::GEAR_RATIO_M2006,
        DjiMotor::ENC_RESOLUTION,
        M_PI / DjiMotor::GEAR_RATIO_M2006);
}

TEST(DjiMotor, getShaftAngleWrappedUnwrapped_M2006_two_full_rotations)
{
    testMotorEncoderValue(
        DjiMotor::GEAR_RATIO_M2006,
        DjiMotor::ENC_RESOLUTION * 2,
        2.0f * M_PI / DjiMotor::GEAR_RATIO_M2006);
}

TEST(DjiMotor, getShaftAngleWrappedUnwrapped_M3508_zero)
{
    testMotorEncoderValue(DjiMotor::GEAR_RATIO_M3508, 0, 0.0f);
}

TEST(DjiMotor, getShaftAngleWrappedUnwrapped_M3508_half_rotation)
{
    testMotorEncoderValue(
        DjiMotor::GEAR_RATIO_M3508,
        DjiMotor::ENC_RESOLUTION / 2,
        0.5f * M_PI / DjiMotor::GEAR_RATIO_M3508);
}

TEST(DjiMotor, getShaftAngleWrappedUnwrapped_M3508_full_rotation)
{
    testMotorEncoderValue(
        DjiMotor::GEAR_RATIO_M3508,
        DjiMotor::ENC_RESOLUTION,
        M_PI / DjiMotor::GEAR_RATIO_M3508);
}

TEST(DjiMotor, getShaftAngleWrappedUnwrapped_M3508_two_full_rotations)
{
    testMotorEncoderValue(
        DjiMotor::GEAR_RATIO_M3508,
        DjiMotor::ENC_RESOLUTION * 2,
        2.0f * M_PI / DjiMotor::GEAR_RATIO_M3508);
}
