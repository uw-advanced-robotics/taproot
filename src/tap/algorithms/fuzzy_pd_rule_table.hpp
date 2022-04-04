
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

#ifndef TAPROOT_FUZZY_PD_RULE_TABLE_HPP_
#define TAPROOT_FUZZY_PD_RULE_TABLE_HPP_

#include <array>
#include "fuzzy_rule_table.hpp"

namespace tap::algorithms {
class FuzzyPDRuleTable : public FuzzyRuleTable<2>
{
public:
    enum FuzzyMembers
    {
        N = 0,  ///< Negative error
        Z = 1,  ///< Zero error
        P = 2,  ///< Positive error
    };

    FuzzyPDRuleTable() {}

    FuzzyPDRuleTable(const std::array<float, 3> &kpParams, const std::array<float, 3> &kdParams)
        : kpArray(kpParams),
          kdArray(kdParams)
    {
    }

    modm::Matrix<float, 2, 1> performFuzzyUpdate(float e, float d) override;

    inline modm::Matrix<float, 2, 1> getFuzzyGains() const override { return fuzzyGains; }

private:
    /**
     * Uses if-else-then rules to perform error fuzzification. 3 Triangle membership functions are
     * used to perform fuzzy classification. These are
     *
     * @param[in] e measured error
     * @param[in] d derivative of the error (wrt time)
     */
    void fuzzification(float e, float d);

    /**
     * Update the fuzzy matrix. Fuzzy matrix multiplication of errorFuzzificationMemberValues and
     * derivativeFuzzificationMemberValues.
     */
    void updateFuzzyMatrix();

    /**
     * Applies defuzzification techniques using the fuzzyMatrix to find the fuzzy gain.
     */
    void defuzzification();

    /**
     * Uses if-else-then rules to perform error fuzzification. 3 Triangle membership functions are
     * used to perform fuzzy classification. These are triangle membership functions defined with
     * maximums at -1, 0, 1 and widths of 1 each (so the negative triangle intersects the axis at 0,
     * etc.).
     *
     * @param[in] e measured error
     * @param[in] d derivative of the error (wrt time)
     */
    static inline void fuzzification(float value, float *fuzzificationMemberValues)
    {
        if (value < 0)
        {
            fuzzificationMemberValues[N] = std::min(-value, 1.0f);
            fuzzificationMemberValues[Z] = std::max(value + 1.0f, 0.0f);
            fuzzificationMemberValues[P] = 0;
        }
        else
        {
            fuzzificationMemberValues[N] = 0;
            fuzzificationMemberValues[Z] = std::max(1.0f - value, 0.0f);
            fuzzificationMemberValues[P] = std::min(value, 1.0f);
        }
    }

private:
    float errorFuzzificationMemberValues[3];
    float derivativeFuzzificationMemberValues[3];
    float fuzzyMatrix[3][3];
    modm::Matrix<float, 2, 1> fuzzyGains;
    std::array<float, 3> kpArray;
    std::array<float, 3> kdArray;
};
}

#endif  // TAPROOT_FUZZY_PD_RULE_TABLE_HPP_
