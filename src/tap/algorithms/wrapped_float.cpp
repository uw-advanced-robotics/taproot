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

#include "wrapped_float.hpp"
#include <assert.h>
#include <cmath>

namespace tap
{
namespace algorithms
{
WrappedFloat::WrappedFloat(const float value, const float lowerBound, const float upperBound)
{
    this->value = value;

    this->lowerBound = lowerBound;
    this->upperBound = upperBound;

    this->validateBounds();
    this->reboundValue();
}

float WrappedFloat::reboundValue()
{
    if (value < lowerBound)
    {
        value = upperBound + fmodf(value - lowerBound, upperBound - lowerBound);
    }
    else if (value > upperBound)
    {
        value = lowerBound + fmodf(value - upperBound, upperBound - lowerBound);
    }

    return value;
}

void WrappedFloat::shiftUp(const float shiftMagnitude)
{
    value += shiftMagnitude;
    reboundValue();
}

void WrappedFloat::shiftDown(const float shiftMagnitude)
{
    value -= shiftMagnitude;
    reboundValue();
}

float WrappedFloat::unwrapBelow() const { return value - (upperBound - lowerBound); }

float WrappedFloat::unwrapAbove() const { return value + (upperBound - lowerBound); }

float WrappedFloat::minDifference(const float otherValue) const
{
    return minDifference(WrappedFloat(otherValue, lowerBound, upperBound));
}

WrappedFloat WrappedFloat::operator- (const WrappedFloat& other) {
    assert (this->getLowerBound() == other.getLowerBound());
    assert (this->getUpperBound() == other.getUpperBound());
    
    if (this->getValue() == other.getValue()) {
        return WrappedFloat(0.0, this->getLowerBound(), this->getUpperBound());
    } else {
        float diff = fabs(this->getValue() - other.getValue());
        if (this->getValue() < other.getValue()) {
            float left_range = this->getValue() - this->getLowerBound();
            float right_range = other.getValue() - this->getUpperBound();
            if (left_range + right_range < diff)
                return WrappedFloat(left_range + right_range, this->getLowerBound(), this->getUpperBound());
            else
                return WrappedFloat(diff, this->getLowerBound(), this->getUpperBound());
        } else {
            float left_range = other.getValue() - this->getLowerBound();
            float right_range = this->getValue() - this->getUpperBound();
            if (left_range + right_range < diff)
                return WrappedFloat(left_range + right_range, this->getLowerBound(), this->getUpperBound());
            else
                return WrappedFloat(diff, this->getLowerBound(), this->getUpperBound());
        }
    }
}

float WrappedFloat::minDifference(const WrappedFloat& otherValue) const
{
    // Find the shortest path to the target (smallest difference)
    float aboveDiff = otherValue.getValue() - this->unwrapAbove();
    float belowDiff = otherValue.getValue() - this->unwrapBelow();
    float stdDiff = otherValue.getValue() - this->getValue();

    float finalDiff;

    if (fabs(aboveDiff) < fabs(belowDiff) && fabs(aboveDiff) < fabs(stdDiff))
    {
        finalDiff = aboveDiff;
    }
    else if (fabs(belowDiff) < fabs(aboveDiff) && fabs(belowDiff) < fabs(stdDiff))
    {
        finalDiff = belowDiff;
    } else {
        finalDiff = stdDiff;
    }

    return finalDiff;
}

void WrappedFloat::shiftBounds(const float shiftMagnitude)
{
    upperBound += shiftMagnitude;
    lowerBound += shiftMagnitude;
    reboundValue();
}

WrappedFloat WrappedFloat::operator+ (const WrappedFloat& other) {
    WrappedFloat added_floats = WrappedFloat(this->value + other.value, this->getLowerBound(), this->getUpperBound());
    added_floats.reboundValue();
    return added_floats;
}

float WrappedFloat::limitValue(
    const WrappedFloat& valueToLimit,
    const float min,
    const float max,
    int* status)
{
    WrappedFloat minContig(min, valueToLimit.lowerBound, valueToLimit.upperBound);
    WrappedFloat maxContig(max, valueToLimit.lowerBound, valueToLimit.upperBound);
    return limitValue(valueToLimit, minContig, maxContig, status);
}

float WrappedFloat::limitValue(
    const WrappedFloat& valueToLimit,
    const WrappedFloat& min,
    const WrappedFloat& max,
    int* status)
{
    if (min.getValue() == max.getValue())
    {
        return valueToLimit.getValue();
    }
    if ((min.getValue() < max.getValue() &&
         (valueToLimit.getValue() > max.getValue() || valueToLimit.getValue() < min.getValue())) ||
        (min.getValue() > max.getValue() && valueToLimit.getValue() > max.getValue() &&
         valueToLimit.getValue() < min.getValue()))
    {
        float targetMinDifference = fabs(valueToLimit.minDifference(min));
        float targetMaxDifference = fabs(valueToLimit.minDifference(max));

        if (targetMinDifference < targetMaxDifference)
        {
            *status = 1;
            return min.getValue();
        }
        else
        {
            *status = 2;
            return max.getValue();
        }
    }
    else
    {
        *status = 0;
        return valueToLimit.getValue();
    }
}

// Getters/Setters ----------------
// Value
float WrappedFloat::getValue() const { return value; }

void WrappedFloat::setValue(const float newValue)
{
    value = newValue;
    this->reboundValue();
}

// Upper bound
float WrappedFloat::getUpperBound() const { return upperBound; }

void WrappedFloat::setUpperBound(const float newValue)
{
    upperBound = newValue;

    this->validateBounds();
    this->reboundValue();
}

// Lower bound
float WrappedFloat::getLowerBound() const { return lowerBound; }

void WrappedFloat::setLowerBound(const float newValue)
{
    lowerBound = newValue;

    this->validateBounds();
    this->reboundValue();
}

void WrappedFloat::validateBounds()
{
    if (lowerBound > upperBound)
    {
        float tmp = this->lowerBound;
        this->lowerBound = this->upperBound;
        this->upperBound = tmp;
    }
}

}  // namespace algorithms

}  //  namespace tap
