/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/algorithms/smooth_pid.hpp"

using namespace tap::algorithms;

static SmoothPidConfig derivativeOnlyConfig()
{
    return {.kp = 0, .ki = 0, .kd = 1, .maxICumulative = 0, .maxOutput = 1000};
}

TEST(SmoothPid, runController_derivative_term_opposes_measurement_rate)
{
    SmoothPid pid(derivativeOnlyConfig());

    EXPECT_FLOAT_EQ(-2, pid.runController(5, 2, 1));
}

TEST(SmoothPid, runControllerDerivateError_derivative_term_follows_error_rate)
{
    SmoothPid pid(derivativeOnlyConfig());
    pid.runControllerDerivateError(1, 1);

    // Error growing by 2 per second should push the output up, like the P term.
    EXPECT_FLOAT_EQ(2, pid.runControllerDerivateError(3, 1));
}

TEST(SmoothPid, runControllerDerivateError_matches_runController_with_measurement_rate)
{
    SmoothPid derivateError(derivativeOnlyConfig());
    SmoothPid measurement(derivativeOnlyConfig());

    // Constant target of 10; measurement goes 4 -> 7 over 0.5 s.
    derivateError.runControllerDerivateError(10 - 4, 0.5f);
    measurement.runController(10 - 4, 0, 0.5f);

    EXPECT_FLOAT_EQ(
        measurement.runController(10 - 7, (7 - 4) / 0.5f, 0.5f),
        derivateError.runControllerDerivateError(10 - 7, 0.5f));
}
