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

#include "fuzzy_pd_rule_table.hpp"

namespace tap::algorithms
{
void FuzzyPDRuleTable::fuzzification(float e, float d)
{
    fuzzification(e, errorFuzzificationMemberValues);
    fuzzification(d, derivativeFuzzificationMemberValues);
}

void FuzzyPDRuleTable::updateFuzzyMatrix()
{
    fuzzyMatrix[N][N] =
        std::min(errorFuzzificationMemberValues[N], derivativeFuzzificationMemberValues[N]);
    fuzzyMatrix[N][Z] =
        std::min(errorFuzzificationMemberValues[N], derivativeFuzzificationMemberValues[Z]);
    fuzzyMatrix[N][P] =
        std::min(errorFuzzificationMemberValues[N], derivativeFuzzificationMemberValues[P]);

    fuzzyMatrix[Z][N] =
        std::min(errorFuzzificationMemberValues[Z], derivativeFuzzificationMemberValues[N]);
    fuzzyMatrix[Z][Z] =
        std::min(errorFuzzificationMemberValues[Z], derivativeFuzzificationMemberValues[Z]);
    fuzzyMatrix[Z][P] =
        std::min(errorFuzzificationMemberValues[Z], derivativeFuzzificationMemberValues[P]);

    fuzzyMatrix[P][N] =
        std::min(errorFuzzificationMemberValues[P], derivativeFuzzificationMemberValues[N]);
    fuzzyMatrix[P][Z] =
        std::min(errorFuzzificationMemberValues[P], derivativeFuzzificationMemberValues[Z]);
    fuzzyMatrix[P][P] =
        std::min(errorFuzzificationMemberValues[P], derivativeFuzzificationMemberValues[P]);
}

void FuzzyPDRuleTable::defuzzification()
{
    float kpSmallWeight = fuzzyMatrix[Z][Z];
    float kpMediumWeight = std::max(
        std::max(
            std::max(fuzzyMatrix[Z][N], fuzzyMatrix[N][Z]),
            std::max(fuzzyMatrix[P][Z], fuzzyMatrix[Z][P])),
        std::max(fuzzyMatrix[N][N], fuzzyMatrix[P][P]));
    float kpLargeWeight = std::max(fuzzyMatrix[N][P], fuzzyMatrix[P][N]);

    if (kpSmallWeight + kpMediumWeight + kpLargeWeight > 0)
    {
        float kp = (kpSmallWeight * kpArray[0] + kpMediumWeight * kpArray[1] +
                    kpLargeWeight * kpArray[2]) /
                   (kpSmallWeight + kpMediumWeight + kpLargeWeight);

        fuzzyGains[0][0] = kp;
    }
    else
    {
        fuzzyGains[0][0] = 0;
    }

    float kdSmallWeight =
        std::max(std::max(fuzzyMatrix[Z][P], fuzzyMatrix[Z][N]), fuzzyMatrix[Z][Z]);

    float kdMediumWeight = std::max(
        std::max(fuzzyMatrix[N][N], fuzzyMatrix[N][Z]),
        std::max(fuzzyMatrix[P][Z], fuzzyMatrix[P][P]));

    float kdLargeWeight = std::max(fuzzyMatrix[Z][N], fuzzyMatrix[Z][P]);

    if (kdSmallWeight + kdMediumWeight + kdLargeWeight)
    {
        float kd = (kdSmallWeight * kdArray[0] + kdMediumWeight * kdArray[1] +
                    kdLargeWeight * kdArray[2]) /
                   (kdSmallWeight + kdMediumWeight + kdLargeWeight);

        fuzzyGains[1][0] = kd;
    }
    else
    {
        fuzzyGains[1][0] = 0;
    }
}

modm::Matrix<float, 2, 1> FuzzyPDRuleTable::performFuzzyUpdate(float e, float d)
{
    fuzzification(e, d);
    updateFuzzyMatrix();
    defuzzification();
    return fuzzyGains;
}
}  // namespace tap::algorithms
