/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAP_ALGORITHMS_LAG_LEAD_HPP_
#define TAP_ALGORITHMS_LAG_LEAD_HPP_

#include "tap/algorithms/discrete_filter.hpp"

namespace tap::algorithms::filter
{
constexpr Coefficients<2, float> phaseLagLeadCoefficients(float k, float z, float p, float ts)
{
    const float a0 = 1.0f;
    const float a1 = (p * ts - 2.0f) / (p * ts + 2.0f);

    const float b0 = k * (z * ts + 2.0f) / (p * ts + 2.0f);
    const float b1 = k * (z * ts - 2.0f) / (p * ts + 2.0f);

    return Coefficients<2, float>{
        .naturalResponseCoefficients = {a0, a1},
        .forcedResponseCoefficients = {b0, b1}};
}
}  // namespace tap::algorithms::filter

#endif  // TAP_ALGORITHMS_LAG_LEAD_HPP_