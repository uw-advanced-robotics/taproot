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
#include "tap/communication/sensors/encoder/wrapped_encoder.hpp"
#include "tap/mock/encoder_interface_mock.hpp"

using namespace tap::encoder;
using namespace tap::algorithms;
using namespace tap::mock;
using namespace testing;

TEST(WrappedEncoder, resetEncoderValue_zeroes_encoder_fields)
{
    WrappedEncoder encoder(false, 8192);

    encoder.updateEncoderValue(1000);
    encoder.resetEncoderValue();
    EXPECT_FLOAT_EQ(0, encoder.getPosition().getUnwrappedValue());
    EXPECT_FLOAT_EQ(0, encoder.getPosition().getWrappedValue());

    encoder.updateEncoderValue(8000);
    encoder.resetEncoderValue();
    EXPECT_FLOAT_EQ(0, encoder.getPosition().getUnwrappedValue());
    EXPECT_FLOAT_EQ(0, encoder.getPosition().getWrappedValue());
}

TEST(WrappedEncoder, moving_relative_to_home_after_zeroed_ok)
{
    tap::arch::clock::ClockStub clock;
    clock.time = 1;
    WrappedEncoder encoder(false, 4);

    encoder.updateEncoderValue(2);
    EXPECT_EQ(Angle(M_PI).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    encoder.resetEncoderValue();
    EXPECT_EQ(Angle(0).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    encoder.updateEncoderValue(3);
    EXPECT_FLOAT_EQ(Angle(M_PI_2).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    encoder.updateEncoderValue(4);
    EXPECT_FLOAT_EQ(Angle(M_PI).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    // We need to make sure that the encoder thinks its going backward
    encoder.updateEncoderValue(3);
    encoder.updateEncoderValue(2);

    encoder.updateEncoderValue(1);
    EXPECT_FLOAT_EQ(Angle(-M_PI_2).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());
}

TEST(WrappedEncoder, inverted_moving_relative_to_home_after_zeroed_ok)
{
    tap::arch::clock::ClockStub clock;
    clock.time = 1;
    WrappedEncoder encoder(true, 4);

    encoder.updateEncoderValue(2);
    EXPECT_EQ(Angle(M_PI_2).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    encoder.resetEncoderValue();
    EXPECT_EQ(Angle(0).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    encoder.updateEncoderValue(3);
    EXPECT_FLOAT_EQ(Angle(-M_PI_2).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    encoder.updateEncoderValue(4);
    EXPECT_FLOAT_EQ(Angle(-M_PI).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    // We need to make sure that the encoder thinks its going backward
    encoder.updateEncoderValue(3);
    encoder.updateEncoderValue(2);

    encoder.updateEncoderValue(1);
    EXPECT_FLOAT_EQ(Angle(M_PI_2).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());
}

TEST(WrappedEncoder, calculates_velocity_correctly)
{
    tap::arch::clock::ClockStub clock;
    WrappedEncoder encoder(false, 4);

    encoder.updateEncoderValue(0);
    EXPECT_FLOAT_EQ(0, encoder.getVelocity());

    clock.time = 1000;
    encoder.updateEncoderValue(1);
    EXPECT_FLOAT_EQ(M_PI_2, encoder.getVelocity());
}

TEST(WrappedEncoder, align_with_updates_values)
{
    tap::arch::clock::ClockStub clock;
    clock.time = 1;

    WrappedEncoder encoder(false, 4);
    EncoderInterfaceMock mock;
    EXPECT_CALL(mock, getPosition).WillRepeatedly(Return(Angle(M_PI)));

    encoder.updateEncoderValue(0);
    EXPECT_EQ(Angle(0).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    encoder.alignWith(&mock);
    EXPECT_EQ(Angle(M_PI).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    encoder.updateEncoderValue(2);
    EXPECT_EQ(Angle(M_TWOPI).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());
}

TEST(WrappedEncoder, gear_ratio_works)
{
    tap::arch::clock::ClockStub clock;
    WrappedEncoder encoder(false, 4, 0.5f);

    encoder.updateEncoderValue(0);
    EXPECT_FLOAT_EQ(0, encoder.getVelocity());
    EXPECT_EQ(Angle(0).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());

    clock.time = 1000;
    encoder.updateEncoderValue(2);
    EXPECT_FLOAT_EQ(M_PI_2, encoder.getVelocity());
    EXPECT_EQ(Angle(M_PI_2).getUnwrappedValue(), encoder.getPosition().getUnwrappedValue());
}