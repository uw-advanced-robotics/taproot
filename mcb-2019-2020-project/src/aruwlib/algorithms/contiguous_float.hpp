#ifndef __CONTIGUOUS_FLOAT_HPP__
#define __CONTIGUOUS_FLOAT_HPP__

namespace aruwlib
{

namespace algorithms
{

/**
 * Wraps a float to allow easy comparison and manipulation of sensor readings
 * that wrap (e.g. -180 to 180).
 * 
 * For bounds 0 - 10, logically:
 *   - 10 + 1 == 1
 *   - 0 - 1 == 9
 *   - 0 == 10
 * 
 * Credit to: https://github.com/Team488/SeriouslyCommonLib/blob/af2ce83a830299a8ab3773bec9b8ccc6ab
 *            5a3367/src/main/java/xbot/common/math/ContiguousDouble.java
 */
class ContiguousFloat {
 public:
    ContiguousFloat(const float& value, const float& lowerBound, const float& upperBound);

    /**
     * Shifts the value so that it still represents the same position but is
     * within the current bounds.
     * 
     * @return the new value for chaining functions
     */
    float reboundValue();

    /**
     * Computes the difference between two values (other - this), accounting for
     * wrapping. Treats the given 'other' value as a number within the same bounds
     * as the current instance.
     * 
     * @param otherValue
     *            the other value to compare against
     * @return the computed difference
     */
    float difference(const float& otherValue) const;

    /**
     * Computes the difference between two values (other - this), accounting for
     * wrapping
     * 
     * @param otherValue
     *            the other value to compare against (must have the same bounds
     *            as the current instance)
     * @return the computed difference
     */
    float difference(const ContiguousFloat& otherValue) const;

    /**
     * Shifts both bounds by the specified amount
     * 
     * @param shiftMagnitude
     *            the amount to add to each bound
     */
    void shiftBounds(const float& shiftMagnitude);

    /**
     * Shifts value by the specified amount (addition)
     * 
     * @param shiftMagnitude
     *            the amount to add to the current value
     */
    void shiftValue(const float& shiftMagnitude);

    // Getters/Setters ----------------
    // Value
    float getValue() const;

    void setValue(const float& newValue);

    // Upper bound
    float getUpperBound() const;

    void setUpperBound(const float& newValue);

    // Lower bound
    float getLowerBound() const;

    void setLowerBound(const float& newValue);

 private:
    float value;

    float lowerBound;
    float upperBound;

    /**
     * Flips the lower and upper bounds if the lower bound is larger than the
     * upper bound.
     */
    void validateBounds();

    /**
     * Calculates a number representing the current value that is higher than
     * (or equal to) the upper bound. Used to make normal numerical comparisons
     * without needing to handle wrap cases.
     * 
     * @return the computed value
     */
    float unwrapAbove() const;

    /**
     * Calculates a number representing the current value that is lower than (or
     * equal to) the lower bound. Used to make normal numerical comparisons
     * without needing to handle wrap cases.
     * 
     * @return the computed value
     */
    float unwrapBelow() const;
};

}  // namespace algorithms

}  // namespace aruwlib

#endif
