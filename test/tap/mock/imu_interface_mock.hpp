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

#ifndef TAPROOT_IMU_INTERFACE_MOCK_HPP_
#define TAPROOT_IMU_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/sensors/imu/imu_interface.hpp"

namespace tap::mock
{
class ImuInterfaceMock : public tap::communication::sensors::imu::ImuInterface
{
public:
    ImuInterfaceMock();
    virtual ~ImuInterfaceMock();

    MOCK_METHOD(const char *, getName, (), (const override));
    MOCK_METHOD(float, getAx, (), (override));
    MOCK_METHOD(float, getAy, (), (override));
    MOCK_METHOD(float, getAz, (), (override));
    MOCK_METHOD(float, getGx, (), (override));
    MOCK_METHOD(float, getGy, (), (override));
    MOCK_METHOD(float, getGz, (), (override));
    MOCK_METHOD(float, getTemp, (), (override));
    MOCK_METHOD(float, getYaw, (), (override));
    MOCK_METHOD(float, getPitch, (), (override));
    MOCK_METHOD(float, getRoll, (), (override));
    MOCK_METHOD(void, requestCalibration, (), (override));
    MOCK_METHOD(ImuState, getImuState, (), (const override));
};
}  // namespace tap::mock

#endif  // TAPROOT_IMU_INTERFACE_MOCK_HPP_
