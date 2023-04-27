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

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/analog_mock.hpp"

using namespace tap::control::chassis;
using tap::Drivers;
using namespace testing;

TEST(PowerLimiter, outputs_no_limiting_when_not_connected)
{
    Drivers drivers;
    tap::mock::AnalogMock mockAnalog;
    EXPECT_CALL(mockAnalog, read).WillRepeatedly(Return(10000));
    tap::communication::sensors::current::AnalogCurrentSensor::Config config =
        {&mockAnalog, static_cast<tap::gpio::Analog::Pin>(0), 1.0, 0.0, 1.0};
    tap::communication::sensors::current::AnalogCurrentSensor sensor(config);
    PowerLimiter limiter(&drivers, &sensor, 60.0, 60.0, 10.0);

    tap::arch::clock::ClockStub clock;
    clock.time = 0;
    limiter.getPowerLimitRatio();
    clock.time = 200;

    EXPECT_FLOAT_EQ(1.0, limiter.getPowerLimitRatio());
}

TEST(PowerLimiter, limits_when_connected)
{
    Drivers drivers;

    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData).WillRepeatedly(Return(true));
    tap::communication::serial::RefSerial::Rx::RobotData robotData = {};
    robotData
        .chassis = {24000, 10000, 240.0, 0, 0.0, 0.0, 0.0, 40};  // Only care about the chassis data
    EXPECT_CALL(drivers.refSerial, getRobotData).WillRepeatedly(ReturnRef(robotData));

    tap::mock::AnalogMock mockAnalog;
    EXPECT_CALL(mockAnalog, read).WillRepeatedly(Return(10000));
    tap::communication::sensors::current::AnalogCurrentSensor::Config config =
        {&mockAnalog, static_cast<tap::gpio::Analog::Pin>(0), 1.0, 0.0, 1.0};
    tap::communication::sensors::current::AnalogCurrentSensor sensor(config);
    PowerLimiter limiter(&drivers, &sensor, 60.0, 60.0, 10.0);

    tap::arch::clock::ClockStub clock;
    clock.time = 0;
    limiter.getPowerLimitRatio();
    clock.time = 200;

    EXPECT_NE(1.0, limiter.getPowerLimitRatio());
}

TEST(PowerLimiter, does_not_limit_with_external_power)
{
    Drivers drivers;
    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData).WillRepeatedly(Return(true));
    tap::communication::serial::RefSerial::Rx::RobotData robotData = {};
    robotData
        .chassis = {24000, 10000, 240.0, 0, 0.0, 0.0, 0.0, 40};  // Only care about the chassis data
    EXPECT_CALL(drivers.refSerial, getRobotData).WillRepeatedly(ReturnRef(robotData));

    tap::mock::AnalogMock mockAnalog;
    EXPECT_CALL(mockAnalog, read).WillRepeatedly(Return(10000));
    tap::communication::sensors::current::AnalogCurrentSensor::Config config =
        {&mockAnalog, static_cast<tap::gpio::Analog::Pin>(0), 1.0, 0.0, 1.0};
    tap::communication::sensors::current::AnalogCurrentSensor sensor(config);
    PowerLimiter limiter(&drivers, &sensor, 60.0, 60.0, 10.0);

    float externalEnergy = 1'000'000.0;
    limiter.setExternalEnergyBuffer(externalEnergy);  // Large external power to ensure no limiting

    tap::arch::clock::ClockStub clock;
    clock.time = 0;
    limiter.getPowerLimitRatio();
    clock.time = 200;

    EXPECT_FLOAT_EQ(1.0, limiter.getPowerLimitRatio());
    EXPECT_NE(externalEnergy, limiter.getExternalEnergyBuffer());
}
