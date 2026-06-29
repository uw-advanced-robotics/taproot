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

#include "tap/communication/sensors/encoder/aligning_encoder.hpp"
#include "tap/mock/encoder_interface_mock.hpp"

using namespace tap::encoder;
using namespace tap::algorithms;
using namespace tap::mock;
using namespace testing;

#define SETUP_ALIGNING_TEST(PRIMARY_ONLINE, SECONDARY_ONLINE, INDEX)    \
    EncoderInterfaceMock mock;                                          \
    EncoderInterfaceMock mock2;                                         \
    std::array<EncoderInterface*, 2> encoders = {&mock, &mock2};        \
    AligningEncoder<2> aligning(encoders, INDEX);                       \
    EXPECT_CALL(mock, isOnline).WillRepeatedly(Return(PRIMARY_ONLINE)); \
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(SECONDARY_ONLINE));

#define SETUP_POSITION_EXPECTS(POS0, POS1)                                \
    EXPECT_CALL(mock, getPosition()).WillRepeatedly(Return(Angle(POS0))); \
    EXPECT_CALL(mock2, getPosition()).WillRepeatedly(Return(Angle(POS1)));

#define SETUP_VELOCITY_EXPECTS(VEL0, VEL1)                         \
    EXPECT_CALL(mock, getVelocity()).WillRepeatedly(Return(VEL0)); \
    EXPECT_CALL(mock2, getVelocity()).WillRepeatedly(Return(VEL1));

TEST(AligningEncoderTests, get_position_averages_both_online)
{
    SETUP_ALIGNING_TEST(true, true, 0);
    SETUP_POSITION_EXPECTS(M_PI, M_PI_2);

    EXPECT_EQ(aligning.getPosition(), Angle(3 * M_PI / 4));
}

TEST(AligningEncoderTests, get_position_averages_secondary_offline_1)
{
    SETUP_ALIGNING_TEST(true, false, 0);
    SETUP_POSITION_EXPECTS(M_PI_2, 0);

    EXPECT_EQ(aligning.getPosition(), Angle(M_PI_2));
}

TEST(AligningEncoderTests, get_position_averages_secondary_offline_2)
{
    SETUP_ALIGNING_TEST(false, true, 1);
    SETUP_POSITION_EXPECTS(0, M_PI_2);

    EXPECT_EQ(aligning.getPosition(), Angle(M_PI_2));
}

TEST(AligningEncoderTests, get_position_averages_primary_offline)
{
    SETUP_ALIGNING_TEST(false, true, 0);
    SETUP_POSITION_EXPECTS(0, M_PI_2);

    EXPECT_EQ(aligning.getPosition(), Angle(0));
}

TEST(AligningEncoderTests, get_velocity_averages_both_online)
{
    SETUP_ALIGNING_TEST(true, true, 0);
    SETUP_VELOCITY_EXPECTS(1.0f, 2.0f);

    EXPECT_FLOAT_EQ(aligning.getVelocity(), 1.5f);
}

TEST(AligningEncoderTests, get_velocity_averages_secondary_offline_1)
{
    SETUP_ALIGNING_TEST(true, false, 0);
    SETUP_VELOCITY_EXPECTS(1.0f, 0.0f);

    EXPECT_FLOAT_EQ(aligning.getVelocity(), 1.0f);
}

TEST(AligningEncoderTests, get_velocity_averages_secondary_offline_2)
{
    SETUP_ALIGNING_TEST(false, true, 1);
    SETUP_VELOCITY_EXPECTS(0.0f, 1.0f);

    EXPECT_FLOAT_EQ(aligning.getVelocity(), 1.0f);
}

TEST(AligningEncoderTests, get_velocity_averages_primary_offline)
{
    SETUP_ALIGNING_TEST(false, true, 0);
    SETUP_POSITION_EXPECTS(0, 2);

    EXPECT_EQ(aligning.getVelocity(), 0.0f);
}