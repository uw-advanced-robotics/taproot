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

/**
 * @brief Header File for Implementation of "Velocity" PIDE Algorithm extending the Smooth Pid
 * Algorithm. Velocity Form PID Equation
 * * \f$
 * CV_n = CV_{n-1} + K_P \Delta E + K_I E \Delta t + K_D \frac{E_n - 2E_{n-1} + E_{n-2}}{\Delta t}
 * \f$
 *
 * where:
 * - \(CV\) = Controlled Variable
 * - \(E\) = Error
 * - \(\Delta t\) = Update time
 * - \(K_P\) = Proportional gain
 * - \(K_I\) = Integral gain
 * - \(K_D\) = Derivative gain
 *
 * @details
 * This pid implementation is different from smooth pid because
 * it uses the error derivative for the p term instead of the error itself,
 * uses the second derivative for the d term and the adds corrections to the previous output.
 *
 * For further information, the link to the paper used to implement this is below.
 * https://literature.rockwellautomation.com/idc/groups/literature/documents/wp/logix-wp008_-en-p.pdf
 *
 */

#ifndef TAPROOT_VELOCITY_SMOOTH_PID_HPP_
#define TAPROOT_VELOCITY_SMOOTH_PID_HPP_

#include <array>
#include <cstdint>

#include "tap/algorithms/extended_kalman.hpp"
#include "tap/algorithms/smooth_pid.hpp"

namespace tap
{
namespace algorithms
{
class VelocitySmoothPid : public SmoothPid
{
public:
    VelocitySmoothPid(const SmoothPidConfig& pidConfig);

    float runController(float error, float errorDerivative, float dt) override;

private:
    std::array<float, 3> pastErrors = {0.0f, 0.0f, 0.0f};
};

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_SMOOTH_PID_HPP_
