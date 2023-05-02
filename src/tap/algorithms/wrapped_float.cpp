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
#include "math_user_utils.hpp"

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
    this->wrapValue();
}

float WrappedFloat::wrapValue()
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

void WrappedFloat::operator+=(WrappedFloat other)
{
    assert(compareFloatClose(this->getLowerBound(), other.getLowerBound(), EPSILON));
    assert(compareFloatClose(this->getUpperBound(), other.getUpperBound(), EPSILON));

    this->value = this->value + other.getValue();
    this->wrapValue();
}

void WrappedFloat::operator-=(WrappedFloat other)
{
    assert(compareFloatClose(this->getLowerBound(), other.getLowerBound(), EPSILON));
    assert(compareFloatClose(this->getUpperBound(), other.getUpperBound(), EPSILON));

    this->value = this->value - other.getValue();
    this->wrapValue();
}

WrappedFloat WrappedFloat::operator+(const WrappedFloat& other) const
{
    assert(compareFloatClose(this->getLowerBound(), other.getLowerBound(), EPSILON));
    assert(compareFloatClose(this->getUpperBound(), other.getUpperBound(), EPSILON));

    WrappedFloat temp = *this;
    temp += other;
    return temp;
}

WrappedFloat WrappedFloat::operator-(const WrappedFloat& other) const
{
    assert(compareFloatClose(this->getLowerBound(), other.getLowerBound(), EPSILON));
    assert(compareFloatClose(this->getUpperBound(), other.getUpperBound(), EPSILON));

    WrappedFloat temp = *this;
    temp -= other;
    return temp;
}

void WrappedFloat::shiftUpInPlace(float shiftMagnitude)
{
    this->value += shiftMagnitude;
    this->wrapValue();
}

void WrappedFloat::shiftDownInPlace(float shiftMagnitude)
{
    this->value -= shiftMagnitude;
    this->wrapValue();
}

WrappedFloat WrappedFloat::shiftUp(float shiftMagnitude) const
{
    WrappedFloat temp = WrappedFloat(this->value + shiftMagnitude, this->lowerBound, this->upperBound);
    temp.wrapValue();
    return temp;
}

WrappedFloat WrappedFloat::shiftDown(float shiftMagnitude) const
{
    WrappedFloat temp = WrappedFloat(this->value - shiftMagnitude, this->lowerBound, this->upperBound);
    temp.wrapValue();
    return temp;
}

WrappedFloat WrappedFloat::minDifference(const float other) const
{
    return minDifference(WrappedFloat(other, lowerBound, upperBound));
}

WrappedFloat WrappedFloat::minDifference(const WrappedFloat& other) const
{
    assert(compareFloatClose(this->getLowerBound(), other.getLowerBound(), EPSILON));
    assert(compareFloatClose(this->getUpperBound(), other.getUpperBound(), EPSILON));

    if (this->getValue() == other.getValue())
    {
        return WrappedFloat(0.0, this->getLowerBound(), this->getUpperBound());
    }
    else
    {
        float diff = fabs(this->getValue() - other.getValue());
        if (this->getValue() < other.getValue())
        {
            float left_range = this->getValue() - this->getLowerBound();
            float right_range = other.getValue() - this->getUpperBound();
            if (left_range + right_range < diff)
                return WrappedFloat(
                    left_range + right_range,
                    this->getLowerBound(),
                    this->getUpperBound());
            else
                return WrappedFloat(diff, this->getLowerBound(), this->getUpperBound());
        }
        else
        {
            float left_range = other.getValue() - this->getLowerBound();
            float right_range = this->getValue() - this->getUpperBound();
            if (left_range + right_range < diff)
                return WrappedFloat(
                    left_range + right_range,
                    this->getLowerBound(),
                    this->getUpperBound());
            else
                return WrappedFloat(diff, this->getLowerBound(), this->getUpperBound());
        }
    }
}

void WrappedFloat::shiftBounds(const float shiftMagnitude)
{
    upperBound += shiftMagnitude;
    lowerBound += shiftMagnitude;
    wrapValue();
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
        WrappedFloat targetMinDifference = valueToLimit.minDifference(min);
        WrappedFloat targetMaxDifference = valueToLimit.minDifference(max);

        if (targetMinDifference.getValue() < targetMaxDifference.getValue())
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
    this->wrapValue();
}

// Upper bound
float WrappedFloat::getUpperBound() const { return upperBound; }

void WrappedFloat::setUpperBound(const float newValue)
{
    upperBound = newValue;

    this->validateBounds();
    this->wrapValue();
}

// Lower bound
float WrappedFloat::getLowerBound() const { return lowerBound; }

void WrappedFloat::setLowerBound(const float newValue)
{
    lowerBound = newValue;

    this->validateBounds();
    this->wrapValue();
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
