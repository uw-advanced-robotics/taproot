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

#ifndef TAPROOT_MPU6500_MOCK_HPP_
#define TAPROOT_MPU6500_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"

namespace tap
{
namespace mock
{
class Mpu6500Mock : public tap::communication::sensors::imu::mpu6500::Mpu6500
{
public:
    Mpu6500Mock(tap::Drivers *drivers);
    virtual ~Mpu6500Mock();

    MOCK_METHOD(void, init, (float, float, float), (override));
    MOCK_METHOD(void, periodicIMUUpdate, (), (override));
    MOCK_METHOD(bool, read, (), (override));
    MOCK_METHOD(ImuState, getImuState, (), (const override));
    MOCK_METHOD(float, getAx, (), (override));
    MOCK_METHOD(float, getAy, (), (override));
    MOCK_METHOD(float, getAz, (), (override));
    MOCK_METHOD(float, getGx, (), (override));
    MOCK_METHOD(float, getGy, (), (override));
    MOCK_METHOD(float, getGz, (), (override));
    MOCK_METHOD(float, getTemp, (), (override));
    MOCK_METHOD(float, getYawDegrees, (), (override));
    MOCK_METHOD(float, getPitchDegrees, (), (override));
    MOCK_METHOD(float, getRollDegrees, (), (override));
    MOCK_METHOD(float, getYawRadians, (), (override));
    MOCK_METHOD(float, getPitchRadians, (), (override));
    MOCK_METHOD(float, getRollRadians, (), (override));
    MOCK_METHOD(float, getTiltAngle, (), (override));
};  // Mpu6500Mock
}  // namespace mock
}  // namespace tap

#endif  //  TAPROOT_MPU6500_MOCK_HPP_
