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

#ifndef TAPROOT_FUZZY_PD_HPP_
#define TAPROOT_FUZZY_PD_HPP_

#include <cstdint>

#include "tap/algorithms/extended_kalman.hpp"

#include "fuzzy_pd_rule_table.hpp"
#include "smooth_pid.hpp"

namespace tap::algorithms
{
struct FuzzyPDConfig
{
    float maxError = 0.0f;
    float maxErrorDerivative = 0.0f;
    FuzzyPDRuleTable fuzzyTable;
};

/**
 * Fuzzy PID controller.
 */
class FuzzyPD : public SmoothPid
{
public:
    FuzzyPD(const FuzzyPDConfig &pidConfig, const SmoothPidConfig &smoothPidConfig);

    float runController(float error, float errorDerivative, float dt) override;

private:
    FuzzyPDConfig config;

    void udpatePidGains(float error, float errorDerivative);
};

}  // namespace tap::algorithms

#endif  // TAPROOT_FUZZY_PD_HPP_
