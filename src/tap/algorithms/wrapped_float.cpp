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
    : wrapped(value),
      lowerBound(lowerBound),
      upperBound(upperBound)
{
    assert(upperBound > lowerBound);

    if (value < lowerBound)
    {
        this->wrapped = upperBound + fmodf(value - upperBound, upperBound - lowerBound);
    }
    else if (value > upperBound)
    {
        this->wrapped = lowerBound + fmodf(value - lowerBound, upperBound - lowerBound);
    }
    this->revolutions = static_cast<int>((value - lowerBound) / (upperBound - lowerBound));
}


void WrappedFloat::operator+=(const WrappedFloat& other)
{
    assert(compareFloatClose(this->upperBound, other.upperBound, EPSILON));
    assert(compareFloatClose(this->upperBound, other.upperBound, EPSILON));

    this->wrapped += other.wrapped;
    if (this->wrapped > this->upperBound)
        this->wrapped -= (this->upperBound - this->lowerBound);
    this->revolutions += other.revolutions;
}

void WrappedFloat::operator-=(const WrappedFloat& other)
{
    assert(compareFloatClose(this->getLowerBound(), other.getLowerBound(), EPSILON));
    assert(compareFloatClose(this->getUpperBound(), other.getUpperBound(), EPSILON));

    this->wrapped -= other.wrapped;
    if (this->wrapped < this->lowerBound)
        this->wrapped += (this->upperBound - this->lowerBound);
    this->revolutions -= other.revolutions;
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

void WrappedFloat::operator+=(float value)
{
    *this += WrappedFloat(value, this->lowerBound, this->upperBound);
}

void WrappedFloat::operator-=(float value)
{
    *this -= WrappedFloat(value, this->lowerBound, this->upperBound);
}

WrappedFloat WrappedFloat::operator+(float value) const
{
    return *this + WrappedFloat(value, this->lowerBound, this->upperBound);
}

WrappedFloat WrappedFloat::operator-(float value) const
{
    return *this - WrappedFloat(value, this->lowerBound, this->upperBound);
}

float WrappedFloat::minDifference(const WrappedFloat& other) const
{
    assert(compareFloatClose(this->getLowerBound(), other.getLowerBound(), EPSILON));
    assert(compareFloatClose(this->getUpperBound(), other.getUpperBound(), EPSILON));

    float halfInterval = (this->upperBound - other.lowerBound) / 2;

    if (this->wrapped >= other.wrapped)
    {
        if (this->wrapped < other.wrapped + halfInterval)
            return this->wrapped - other.wrapped;
        else
            return this->lowerBound - other.wrapped + this->wrapped - this->upperBound;
    }
    else
    {
        if (other.wrapped < this->wrapped + halfInterval)
            return other.wrapped - this->wrapped;
        else
            return this->lowerBound - this->wrapped + other.wrapped - this->upperBound;
    }
}

WrappedFloat WrappedFloat::minInterpolate(const WrappedFloat& other, const float alpha)
{
    assert(compareFloatClose(this->lowerBound, other.lowerBound, EPSILON)); 
    assert(compareFloatClose(this->upperBound, other.upperBound, EPSILON)); 
    float halfInterval = (this->upperBound - other.lowerBound) / 2;
    float rawInterpolation = this->wrapped * alpha + other.wrapped * (1 - alpha);

    if (abs(this->wrapped - other.wrapped) <= halfInterval)
    {
        return WrappedFloat(rawInterpolation, this->lowerBound, this->upperBound);
    }
    else
    {
        if (rawInterpolation > this->lowerBound + halfInterval)
            return WrappedFloat(rawInterpolation - halfInterval, this->lowerBound, this->upperBound);
        else
            return WrappedFloat(rawInterpolation + halfInterval, this->lowerBound, this->upperBound);
    }
}

void WrappedFloat::shiftBounds(const float shiftMagnitude)
{
    upperBound += shiftMagnitude;
    lowerBound += shiftMagnitude;
    wrapValue();
}

}  // namespace algorithms

}  //  namespace tap
