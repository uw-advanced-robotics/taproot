/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "turret_pid.hpp"

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/clock.hpp>

using namespace aruwlib::algorithms;

namespace aruwsrc
{
namespace algorithms
{
float TurretPid::runController(float error, float errorDerivative)
{
    // p
    currErrorP = kp * proportionalKalman.filterData(error);
    // i
    currErrorI = limitVal<float>(
        currErrorI + ki * proportionalKalman.getLastFiltered(),
        -maxICumulative,
        maxICumulative);
    // d
    currErrorD = -kd * derivativeKalman.filterData(errorDerivative);
    // total
    output = limitVal<float>(currErrorP + currErrorI + currErrorD, -maxOutput, maxOutput);
    return output;
}

float TurretPid::runControllerDerivateError(float error, float dt)
{
    float errorDerivative = error / dt;
    previousTimestamp = aruwlib::arch::clock::getTimeMilliseconds();
    return runController(error, errorDerivative);
}

float TurretPid::getOutput() { return output; }

void TurretPid::reset()
{
    this->output = 0.0f;
    this->currErrorP = 0.0f;
    this->currErrorI = 0.0f;
    this->currErrorD = 0.0f;
    this->derivativeKalman.reset();
    this->proportionalKalman.reset();
}

}  // namespace algorithms

}  // namespace aruwsrc
