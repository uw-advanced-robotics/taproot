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
                                                                           \
    EXPECT_CALL(mock, isOnline).WillRepeatedly(Return(PRIMARY_ONLINE));    \
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(SECONDARY_ONLINE)); \
    EXPECT_CALL(mock2, alignWithEncoder(&mock)).Times(PRIMARY_ONLINE &SECONDARY_ONLINE)

TEST(FallbackEncoderTests, get_position_averages_target_online)
{
    SETUP_TEST(true, true);
    AligningEncoder<2> aligning1(encoders, 0);
    AligningEncoder<2> aligning2(encoders, 1);

    EXPECT_CALL(mock, getPosition).WillOnce(Return(Angle(M_PI)));
    EXPECT_CALL(mock2, getPosition).WillOnce(Return(Angle(M_PI_2)));

    EXPECT_EQ(aligning1.getPosition(), Angle(M_PI));
    EXPECT_EQ(aligning2.getPosition(), Angle(M_PI_2));
}

TEST(FallbackEncoderTests, get_position_averages_target_offline)
{
    SETUP_TEST(false, true);
    AligningEncoder<2> aligning(encoders, 0);

    EXPECT_CALL(mock, getPosition).Times(0);
    EXPECT_CALL(mock2, getPosition).WillOnce(Return(Angle(M_PI_2)));

    EXPECT_EQ(aligning.getPosition(), ANGLE(M_PI_2));
}

TEST(FallbackEncoderTests, get_velocity_averages_target_online)
{
    SETUP_TEST(true, true);
    AligningEncoder<2> aligning1(encoders, 0);
    AligningEncoder<2> aligning2(encoders, 1);

    EXPECT_CALL(mock, getVelocity).WillOnce(Return(1));
    EXPECT_CALL(mock2, getPosition).WillOnce(Return(2));

    EXPECT_EQ(aligning1, getVelocity).WillOnce(Return(1));
    EXPECT_EQ(aligning2, getVelocity).WillOnce(Return(2));
}

TEST(FallbackEncoderTests, get_velocity_averages_target_offline)
{
    SETUP_TEST(false, true);
    AligningEncoder<2> aligning(encoders, 0);

    EXPECT_CALL(mock, getVelocity).Times(0);
    EXPECT_CALL(mock2, getPosition).WillOnce(Return(1));

    EXPECT_EQ(aligning, getVelocity).WillOnce(Return(1));
}