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

#ifndef MPU6500_MOCK_HPP_
#define MPU6500_MOCK_HPP_

#include <aruwlib/communication/sensors/mpu6500/mpu6500.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class Mpu6500Mock : public aruwlib::sensors::Mpu6500
{
public:
    Mpu6500Mock(aruwlib::Drivers *drivers) : aruwlib::sensors::Mpu6500(drivers) {}
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, read, (), (override));
    MOCK_METHOD(bool, initialized, (), (const override));
    MOCK_METHOD(float, getAx, (), (const override));
    MOCK_METHOD(float, getAy, (), (const override));
    MOCK_METHOD(float, getAz, (), (const override));
    MOCK_METHOD(float, getGx, (), (const override));
    MOCK_METHOD(float, getGy, (), (const override));
    MOCK_METHOD(float, getGz, (), (const override));
    MOCK_METHOD(float, getTemp, (), (const override));
    MOCK_METHOD(float, getYaw, (), (override));
    MOCK_METHOD(float, getPitch, (), (override));
    MOCK_METHOD(float, getRoll, (), (override));
    MOCK_METHOD(float, getTiltAngle, (), (override));
};  // Mpu6500Mock
}  // namespace mock
}  // namespace aruwlib

#endif  //  MPU6500_MOCK_HPP_
