#include <math.h>
#include "contiguous_float.hpp"

namespace aruwlib
{

namespace algorithms
{

ContiguousFloat::ContiguousFloat(
    const float& value,
    const float& lowerBound,
    const float& upperBound
) {
    this->value = value;

    this->lowerBound = lowerBound;
    this->upperBound = upperBound;

    this->validateBounds();
    this->reboundValue();
}

float ContiguousFloat::reboundValue() {
    if (value < lowerBound) {
        value = upperBound
            + fmod(value - lowerBound, upperBound - lowerBound);
    } else if (value > upperBound) {
        value = lowerBound
            + fmod(value - upperBound, upperBound - lowerBound);
    }

    return value;
}

float ContiguousFloat::unwrapBelow() const {
    return lowerBound - (upperBound - value);
}

float ContiguousFloat::unwrapAbove() const {
    return upperBound + (value - lowerBound);
}

float ContiguousFloat::difference(const float& otherValue) {
    return difference(ContiguousFloat(otherValue, lowerBound, upperBound));
}

float ContiguousFloat::difference(const ContiguousFloat& otherValue) {
    // Find the shortest path to the target (smallest difference)
    float aboveDiff = otherValue.getValue() - this->unwrapAbove();
    float belowDiff = otherValue.getValue() - this->unwrapBelow();
    float stdDiff = otherValue.getValue() - this->getValue();

    float finalDiff = stdDiff;

    if (
        fabs(aboveDiff) < fabs(belowDiff)
        && fabs(aboveDiff) < fabs(stdDiff))
    {
        finalDiff = aboveDiff;
    }
    else if (
        fabs(belowDiff) < fabs(aboveDiff)
        && fabs(belowDiff) < fabs(stdDiff))
    {
        finalDiff = belowDiff;
    }

    return finalDiff;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void ContiguousFloat::shiftBounds(const float& shiftMagnitude) {
    upperBound += shiftMagnitude;
    lowerBound += shiftMagnitude;
    reboundValue();
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void ContiguousFloat::shiftValue(const float& shiftMagnitude) {
    value += shiftMagnitude;
    reboundValue();
}

// Getters/Setters ----------------
// Value
// cppcheck-suppress unusedFunction //TODO Remove lint suppression
float ContiguousFloat::getValue() const {
    return value;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void ContiguousFloat::setValue(const float& newValue) {
    value = newValue;
    this->reboundValue();
}

// Upper bound
// cppcheck-suppress unusedFunction //TODO Remove lint suppression
float ContiguousFloat::getUpperBound() const {
    return upperBound;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void ContiguousFloat::setUpperBound(const float& newValue) {
    upperBound = newValue;

    this->validateBounds();
    this->reboundValue();
}

// Lower bound
// cppcheck-suppress unusedFunction //TODO Remove lint suppression
float ContiguousFloat::getLowerBound() const {
    return lowerBound;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void ContiguousFloat::setLowerBound(const float& newValue) {
    lowerBound = newValue;

    this->validateBounds();
    this->reboundValue();
}

void ContiguousFloat::validateBounds() {
    if (lowerBound > upperBound) {
        float tmp = this->lowerBound;
        this->lowerBound = this->upperBound;
        this->upperBound = tmp;
    }
}

}  // namespace algorithms

}  //  namespace aruwlib
