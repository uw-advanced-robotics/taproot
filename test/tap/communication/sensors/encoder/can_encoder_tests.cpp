/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/drivers.hpp"

#include "modm/architecture/interface/can_message.hpp"

using namespace tap;
using namespace tap::encoder;
using namespace tap::algorithms;
using namespace tap::mock;
using namespace testing;

TEST(CanEncoderTests, encoder_init_attaches_to_rx_handler)
{
    Drivers drivers;

    CanEncoder encoder(&drivers, CanEncoderId::ID0, tap::can::CanBus::CAN_BUS1);

    EXPECT_CALL(drivers.canRxHandler, attachReceiveHandler(&encoder));
    encoder.initialize();
}

TEST(CanEncoderTests, encoder_timeout_is_offline)
{
    tap::arch::clock::ClockStub clock;
    Drivers drivers;

    CanEncoder encoder(&drivers, CanEncoderId::ID0, tap::can::CanBus::CAN_BUS1);

    ASSERT_TRUE(encoder.isOnline());
    clock.time = 50;
    ASSERT_TRUE(encoder.isOnline());
    clock.time = 101;
    ASSERT_FALSE(encoder.isOnline());
    modm::can::Message message{};
    memset(message.data, 0, 8);
    encoder.processMessage(message);
    clock.time = 200;
    ASSERT_TRUE(encoder.isOnline());
    clock.time = 201;
    ASSERT_FALSE(encoder.isOnline());
}

TEST(CanEncoderTests, encoder_reads_correctly)
{
    Drivers drivers;

    CanEncoder encoder(&drivers, CanEncoderId::ID0, tap::can::CanBus::CAN_BUS1);

    modm::can::Message message(CanEncoderId::ID0, 4);
    uint16_t data[] = {CanEncoder::ENCODER_RESOLUTION / 4, 1000};
    memcpy(message.data, &data, 4);

    encoder.processMessage(message);

    EXPECT_EQ(
        encoder.getEncoder(),
        tap::algorithms::WrappedFloat(
            CanEncoder::ENCODER_RESOLUTION / 4,
            0,
            CanEncoder::ENCODER_RESOLUTION));
    EXPECT_EQ(encoder.getPosition(), Angle(M_PI_2));

    EXPECT_EQ(encoder.getGauss(), 1000);
}