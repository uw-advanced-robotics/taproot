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
    : value_(value),
      lowerBound_(lowerBound),
      upperBound_(upperBound)
{
    assert(upperBound_ > lowerBound_);

    this->wrapValue();
}


void WrappedFloat::operator+=(const WrappedFloat& other)
{
    assert(compareFloatClose(this->getLowerBound(), other.getLowerBound(), EPSILON));
    assert(compareFloatClose(this->getUpperBound(), other.getUpperBound(), EPSILON));

    this->value_ = this->value_ + other.value_;
    this->wrapValue();
}

void WrappedFloat::operator-=(const WrappedFloat& other)
{
    assert(compareFloatClose(this->getLowerBound(), other.getLowerBound(), EPSILON));
    assert(compareFloatClose(this->getUpperBound(), other.getUpperBound(), EPSILON));

    this->value_ = this->value_ - other.value_;
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
    this->value_ += shiftMagnitude;
    this->wrapValue();
}

void WrappedFloat::shiftDownInPlace(float shiftMagnitude)
{
    this->value_ -= shiftMagnitude;
    this->wrapValue();
}

WrappedFloat WrappedFloat::shiftUp(float shiftMagnitude) const
{
    WrappedFloat temp = WrappedFloat(this->value_ + shiftMagnitude, this->lowerBound_, this->upperBound_);
    return temp;
}

WrappedFloat WrappedFloat::shiftDown(float shiftMagnitude) const
{
    WrappedFloat temp = WrappedFloat(this->value_ - shiftMagnitude, this->lowerBound_, this->upperBound_);
    return temp;
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
        float diff = fabs(this->value_ - other.value_);
        if (this->value_ < other.value_)
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
    upperBound_ += shiftMagnitude;
    lowerBound_ += shiftMagnitude;
    wrapValue();
}

/* Private methods */

void WrappedFloat::wrapValue()
{
    if (value_ < lowerBound_)
    {
        value_ = upperBound_ + fmodf(value_ - lowerBound_, upperBound_ - lowerBound_);
    }
    else if (value_ > upperBound_)
    {
        value_ = lowerBound_ + fmodf(value_ - upperBound_, upperBound_ - lowerBound_);
    }
}

}  // namespace algorithms

}  //  namespace tap
