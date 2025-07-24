#ifndef TAP_ALGORITHMS_LAG_LEAD_HPP_
#define TAP_ALGORITHMS_LAG_LEAD_HPP_

#include "tap/algorithms/discrete_filter.hpp"

namespace tap::algorithms::filter
{
constexpr Coefficients<2, float> PhaseLagLeadCoefficients(float k, float z, float p, float ts)
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