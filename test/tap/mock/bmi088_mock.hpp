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

#ifndef BMI088_MOCK_HP_
#define BMI088_MOCK_HP_

#include <gmock/gmock.h>

#include "tap/communication/sensors/bmi088/bmi088.hpp"

namespace tap::mock
{
class Bmi088Mock : public tap::communication::sensors::bmi088::Bmi088
{
public:
    Bmi088Mock(tap::Drivers *drivers);
    virtual ~Bmi088Mock();

    MOCK_METHOD(void, initiailze, (), (override));
    MOCK_METHOD(void, periodicIMUUpdate, (), (override));
    MOCK_METHOD(ImuState, getImuState, (), (const override));
    MOCK_METHOD(void, requestRecalibration, (), (override));
    MOCK_METHOD(float, getYaw, (), (override));
    MOCK_METHOD(float, getPitch, (), (override));
    MOCK_METHOD(float, getRoll, (), (override));
    MOCK_METHOD(float, getGx, (), (const override));
    MOCK_METHOD(float, getGy, (), (const override));
    MOCK_METHOD(float, getGz, (), (const override));
    MOCK_METHOD(float, getAx, (), (const override));
    MOCK_METHOD(float, getAy, (), (const override));
    MOCK_METHOD(float, getAz, (), (const override));
    MOCK_METHOD(float, getTemp, (), (const override));
};
}  // namespace tap::mock

#endif  // BMI088_MOCK_HP_
