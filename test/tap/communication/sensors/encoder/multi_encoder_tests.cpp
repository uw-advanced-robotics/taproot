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

TEST(MultiEncoderTests, constructing_multi_encoder_succeeds_when_first_encoder_is_not_null)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 1> encoders = {&mock};
    MultiEncoder<1> multi(encoders);

    std::array<EncoderInterface *, 2> encoders2 = {&mock, &mock2};
    MultiEncoder<2> multi2(encoders2);
}

TEST(MultiEncoderTests, constructing_multi_encoder_fails_when_first_encoder_is_null_DEATH)
{
    std::array<EncoderInterface *, 1> encoders = {nullptr};
    ASSERT_DEATH({ MultiEncoder<1> multi(encoders); }, ".*");
}

TEST(MultiEncoderTests, initialize_calls_internal_initialize)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 3> encoders = {&mock, &mock2, nullptr};
    MultiEncoder<3> multi(encoders);

    EXPECT_CALL(mock, initialize);
    EXPECT_CALL(mock2, initialize);

    multi.initialize();
}

TEST(MultiEncoderTests, is_online_when_primary_is_online)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};
    MultiEncoder<2> multi(encoders);

    EXPECT_CALL(mock, isOnline).WillRepeatedly(Return(true));
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(false));

    EXPECT_TRUE(multi.isOnline());
}

TEST(MultiEncoderTests, is_online_when_all_internal_are_online)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};
    MultiEncoder<2> multi(encoders);

    EXPECT_CALL(mock, isOnline).WillRepeatedly(Return(true));
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(true));
    EXPECT_CALL(mock2, alignWith(&mock)).Times(1);

    EXPECT_TRUE(multi.isOnline());
}

TEST(MultiEncoderTests, is_offline_when_all_internal_are_offline)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};
    MultiEncoder<2> multi(encoders);

    EXPECT_CALL(mock, isOnline).WillRepeatedly(Return(false));
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(false));

    EXPECT_FALSE(multi.isOnline());
}

TEST(MultiEncoderTests, is_offline_when_primary_is_offline)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};
    MultiEncoder<2> multi(encoders);

    EXPECT_CALL(mock, isOnline).WillRepeatedly(Return(false));
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(true));

    EXPECT_FALSE(multi.isOnline());
}

TEST(MultiEncoderTests, is_offline_when_primary_goes_offline_and_secondary_is_offline)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};
    MultiEncoder<2> multi(encoders);

    EXPECT_CALL(mock, isOnline)
        .WillOnce(Return(true))
        .WillOnce(Return(true))    // First call
        .WillOnce(Return(false));  // Second call
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(false));

    EXPECT_TRUE(multi.isOnline());
    EXPECT_FALSE(multi.isOnline());
}

TEST(MultiEncoderTests, is_online_when_primary_goes_offline_and_secondary_is_online)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};
    MultiEncoder<2> multi(encoders);

    EXPECT_CALL(mock, isOnline)
        .WillOnce(Return(true))
        .WillOnce(Return(true))    // First call
        .WillOnce(Return(false));  // Second call
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(true));
    EXPECT_CALL(mock2, alignWith(&mock)).Times(1);

    EXPECT_TRUE(multi.isOnline());
    EXPECT_TRUE(multi.isOnline());
}

TEST(MultiEncoderTests, is_offline_when_primary_goes_offline_and_secondary_goes_online)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};
    MultiEncoder<2> multi(encoders);

    EXPECT_CALL(mock, isOnline)
        .WillOnce(Return(true))
        .WillOnce(Return(true))    // First call
        .WillOnce(Return(false));  // Second call
    EXPECT_CALL(mock2, isOnline)
        .WillOnce(Return(false))        // First call
        .WillRepeatedly(Return(true));  // Second call. Not called because of short circuiting.

    EXPECT_TRUE(multi.isOnline());
    EXPECT_FALSE(multi.isOnline());
}

TEST(MultiEncoderTests, is_offline_when_primary_and_secondary_go_offline)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};
    MultiEncoder<2> multi(encoders);

    EXPECT_CALL(mock, isOnline)
        .WillOnce(Return(true))
        .WillOnce(Return(true))    // First call
        .WillOnce(Return(false));  // Second call
    EXPECT_CALL(mock2, isOnline)
        .WillOnce(Return(true))          // First call
        .WillRepeatedly(Return(false));  // Second call.
    EXPECT_CALL(mock2, alignWith(&mock)).Times(1);

    EXPECT_TRUE(multi.isOnline());
    EXPECT_FALSE(multi.isOnline());
}

TEST(MultiEncoderTests, is_online_when_primary_goes_off_and_online_and_secondary_is_online)
{
    EncoderInterfaceMock mock;
    EncoderInterfaceMock mock2;

    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};
    MultiEncoder<2> multi(encoders);

    EXPECT_CALL(mock, isOnline)
        .WillOnce(Return(true))
        .WillOnce(Return(true))   // First call
        .WillOnce(Return(false))  // Second call
        .WillOnce(Return(true))
        .WillOnce(Return(true));  // Third call
    EXPECT_CALL(mock, alignWith(&mock2)).Times(1);

    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(true));
    EXPECT_CALL(mock2, alignWith(&mock)).Times(1);

    EXPECT_TRUE(multi.isOnline());
    EXPECT_TRUE(multi.isOnline());
    EXPECT_TRUE(multi.isOnline());
}

#define SETUP_TEST(PRIMARY_ONLINE, SECONDARY_ONLINE)                       \
    EncoderInterfaceMock mock;                                             \
    EncoderInterfaceMock mock2;                                            \
                                                                           \
    std::array<EncoderInterface *, 2> encoders = {&mock, &mock2};          \
    MultiEncoder<2> multi(encoders);                                       \
                                                                           \
    EXPECT_CALL(mock, isOnline).WillRepeatedly(Return(PRIMARY_ONLINE));    \
    EXPECT_CALL(mock2, isOnline).WillRepeatedly(Return(SECONDARY_ONLINE)); \
    EXPECT_CALL(mock2, alignWith(&mock)).Times(PRIMARY_ONLINE &SECONDARY_ONLINE)

TEST(MultiEncoderTests, get_position_averages_positions)
{
    SETUP_TEST(true, true);

    EXPECT_CALL(mock, getPosition).WillOnce(Return(Angle(M_PI)));
    EXPECT_CALL(mock2, getPosition).WillOnce(Return(Angle(0)));

    EXPECT_EQ(multi.getPosition(), Angle(M_PI_2));
}

TEST(MultiEncoderTests, get_position_averages_online_positions)
{
    SETUP_TEST(true, false);

    EXPECT_CALL(mock, getPosition).WillOnce(Return(WrappedFloat(M_PI, 0, M_TWOPI)));
    EXPECT_CALL(mock2, getPosition).Times(0);

    EXPECT_EQ(multi.getPosition(), Angle(M_PI));
}

TEST(MultiEncoderTests, get_position_averages_no_online_positions_without_primary)
{
    SETUP_TEST(false, true);

    EXPECT_CALL(mock, getPosition).Times(0);
    EXPECT_CALL(mock2, getPosition).Times(0);

    EXPECT_EQ(multi.getPosition(), Angle(0));
}

TEST(MultiEncoderTests, get_position_averages_offline_positions)
{
    SETUP_TEST(false, false);

    EXPECT_CALL(mock, getPosition).Times(0);
    EXPECT_CALL(mock2, getPosition).Times(0);

    EXPECT_EQ(multi.getPosition(), Angle(0));
}

TEST(MultiEncoderTests, get_velocity_averages_velocities)
{
    SETUP_TEST(true, true);

    EXPECT_CALL(mock, getVelocity).WillOnce(Return(1));
    EXPECT_CALL(mock2, getVelocity).WillOnce(Return(0));

    EXPECT_FLOAT_EQ(multi.getVelocity(), 0.5);
}

TEST(MultiEncoderTests, get_velocity_averages_online_velocities)
{
    SETUP_TEST(true, false);

    EXPECT_CALL(mock, getVelocity).WillOnce(Return(1));
    EXPECT_CALL(mock2, getVelocity).Times(0);

    EXPECT_FLOAT_EQ(multi.getVelocity(), 1);
}

TEST(MultiEncoderTests, get_velocity_averages_no_online_velocities_without_primary)
{
    SETUP_TEST(false, true);

    EXPECT_CALL(mock, getVelocity).Times(0);
    EXPECT_CALL(mock2, getVelocity).Times(0);

    EXPECT_FLOAT_EQ(multi.getVelocity(), 0);
}

TEST(MultiEncoderTests, get_velocity_averages_offline)
{
    SETUP_TEST(false, false);

    EXPECT_CALL(mock, getVelocity).Times(0);
    EXPECT_CALL(mock2, getVelocity).Times(0);

    EXPECT_FLOAT_EQ(multi.getVelocity(), 0);
}

TEST(MultiEncoderTests, reset_encoder_value_resets_encoders)
{
    SETUP_TEST(true, true);

    EXPECT_CALL(mock, resetEncoderValue).Times(1);
    EXPECT_CALL(mock2, resetEncoderValue).Times(1);

    multi.resetEncoderValue();
}

TEST(MultiEncoderTests, align_with_aligns_encoders)
{
    SETUP_TEST(false, false);

    EncoderInterfaceMock align;

    EXPECT_CALL(mock, alignWith(&align)).Times(1);
    EXPECT_CALL(mock2, alignWith(&align)).Times(1);

    multi.alignWith(&align);
}