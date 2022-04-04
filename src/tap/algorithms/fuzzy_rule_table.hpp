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

#ifndef TAPROOT_FUZZY_RULE_TABLE_HPP_
#define TAPROOT_FUZZY_RULE_TABLE_HPP_

#include "modm/math/matrix.hpp"

namespace tap::algorithms
{
template <int OUTPUTS>
class FuzzyRuleTable
{
    virtual modm::Matrix<float, OUTPUTS, 1> performFuzzyUpdate(float e, float d) = 0;
    virtual inline modm::Matrix<float, OUTPUTS, 1> getFuzzyGains() const = 0;
};
}  // namespace tap::algorithms

#endif  // TAPROOT_FUZZY_RULE_TABLE_HPP_
