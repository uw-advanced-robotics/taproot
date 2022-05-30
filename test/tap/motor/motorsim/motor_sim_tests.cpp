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

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/motor/motorsim/motor_sim.hpp"

using namespace testing;
using namespace tap::motor::motorsim;

class MotorSimTest : public Test
{
protected:
    static constexpr MotorSim::Config TEST_CONFIG = {
        .maxInputMag = 10'000,
        .maxencoder = 1'000,
        .maxCurrent = 20,
        .currentLim = 10,
        .maxW = 100,
        .kt = 0.5f,
        .wtGrad = 100,
    };

    MotorSimTest() : motorsim(TEST_CONFIG) {}

    MotorSim motorsim;

    tap::arch::clock::ClockStub clock;
};

TEST_F(MotorSimTest, setInput_updates_motor_current)
{
    motorsim.setMotorInput(TEST_CONFIG.maxInputMag / 2);
    EXPECT_EQ(TEST_CONFIG.maxCurrent / 2, motorsim.getCurrent());
}

TEST_F(MotorSimTest, reset_resets_encoder_rpm_input)
{
    motorsim.setMotorInput(TEST_CONFIG.maxInputMag);
    clock.time += 1'000;
    motorsim.update();

    EXPECT_NE(0, motorsim.getEnc());
    EXPECT_NE(0, motorsim.getRPM());
    EXPECT_NE(0, motorsim.getCurrent());

    motorsim.reset();

    EXPECT_EQ(0, motorsim.getEnc());
    EXPECT_EQ(0, motorsim.getRPM());
    EXPECT_EQ(0, motorsim.getCurrent());
}

TEST_F(MotorSimTest, update_positive_input_shaft_rpm_positive_encoder_increases)
{
    motorsim.setMotorInput(TEST_CONFIG.maxInputMag);
    clock.time += 1'000;
    motorsim.update();

    EXPECT_GT(motorsim.getRPM(), 0);
    EXPECT_GT(motorsim.getEnc(), 0);
}

TEST_F(MotorSimTest, update_negative_input_shaft_rpm_negative)
{
    motorsim.setMotorInput(-TEST_CONFIG.maxInputMag);
    clock.time += 1'000;
    motorsim.update();

    EXPECT_LT(motorsim.getRPM(), 0);
}

TEST_F(MotorSimTest, update_higher_torque_slower_rpm)
{
    motorsim.setMotorInput(TEST_CONFIG.maxInputMag);

    clock.time += 1'000;
    motorsim.update();
    auto fastRPM = motorsim.getRPM();

    motorsim.setLoad(5);

    clock.time += 1'000;
    motorsim.update();
    EXPECT_GT(fastRPM, motorsim.getRPM());
}
