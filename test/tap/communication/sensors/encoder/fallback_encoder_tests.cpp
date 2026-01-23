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

#include "tap/communication/sensors/encoder/multi_encoder.hpp"
#include "tap/mock/encoder_interface_mock.hpp"

using namespace tap::encoder;
using namespace tap::algorithms;
using namespace tap::mock;
using namespace testing;

#define SETUP_TEST(PRIMARY_ONLINE, SECONDARY_ONLINE)                       \
    EncoderInterfaceMock mock;                                             \
    EncoderInterfaceMock mock2;                                            \
                                                                           \
    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};          \
    FallbackEncoder<2> fallback(encoders);                                 \
                                                                           \
    EXPECT_CALL(mock, isOnline).WillRepeatedly(Return(PRIMARY_ONLINE));    \
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(SECONDARY_ONLINE)); \
    EXPECT_CALL(mock2, alignWith(&mock)).Times(PRIMARY_ONLINE &SECONDARY_ONLINE)

TEST(FallbackEncoderTests, get_position_averages_main_online)
{
    SETUP_TEST(true, true);

    EXPECT_CALL(mock, getPosition).WillOnce(Return(Angle(M_PI_2)));
    EXPECT_CALL(mock2, getPosition).WillOnce(Return(Angle(M_PI)));

    EXPECT_EQ(mock, getPosition).willRepeatedly(Return(M_PI_2));
}

TEST(FallbackEncoderTests, get_position_averages_main_offline)
{
    SETUP_TEST(false, true);

    EXPECT_CALL(mock, getPosition).Times(0);
    EXPECT_CALL(mock2, getPosition).WillOnce(Return(Angle(M_PI)));

    EXPECT_EQ(mock, getPosition).willRepeatedly(Return(M_PI));
}

TEST(FallbackEncoderTests, get_velocity_averages_main_online)
{
    SETUP_TEST(true, true);

    EXPECT_CALL(mock, getVelocity).WillOnce(Return(2));
    EXPECT_CALL(mock2, getPosition).WillOnce(Return(1));

    EXPECT_EQ(mock, getPosition).willRepeatedly(Return(2));
}

TEST(FallbackEncoderTests, get_velocity_averages_main_offline)
{
    SETUP_TEST(false, true);

    EXPECT_CALL(mock, getVelocity).Times(0);
    EXPECT_CALL(mock2, getPosition).WillOnce(Return(1));

    EXPECT_EQ(mock, getPosition).willRepeatedly(Return(1));
}