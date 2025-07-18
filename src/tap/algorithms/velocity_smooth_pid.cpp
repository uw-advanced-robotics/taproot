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

// This pid implementation is different from smooth pid 
// bc it uses the error derivative for the p term instead of the error itself,
// uses the second derivative for the d term and the adds corrections to the previous output.

#include "velocity_smooth_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace tap
{
namespace algorithms
{
VelocitySmoothPid::VelocitySmoothPid(const SmoothPidConfig& pidConfig) : SmoothPid(pidConfig) {}

float VelocitySmoothPid::runController(float error, float errorDerivative, float dt)
{
    pastErrors[0] = pastErrors[1];
    pastErrors[1] = pastErrors[2];
    pastErrors[2] = proportionalKalman.filterData(error);

    if (abs(error) < config.errDeadzone)
    {
        error = 0.0f;
    }

    // p
    currErrorP = config.kp * derivativeKalman.filterData(errorDerivative);
    // i
    currErrorI = limitVal<float>(
        config.ki * proportionalKalman.getLastFiltered() * dt,
        -config.maxICumulative,
        config.maxICumulative);
    // d
    currErrorD = -config.kd * (pastErrors[2] - 2 * pastErrors[1] + pastErrors[0]) / dt;
    if (fabs(error) < config.errorDerivativeFloor)
    {
        // the error is less than some amount, so round derivative output to 0
        // done to avoid high frequency control oscilations in some systems
        currErrorD = 0.0f;
    }
    // total
    output = limitVal<float>(
        currErrorP + currErrorI + currErrorD + output,
        -config.maxOutput,
        config.maxOutput);
    return output;
}

}  // namespace algorithms

}  // namespace tap
