#include "ramp.hpp"

#include <math.h>

#include "math_user_utils.hpp"

namespace aruwlib
{
namespace algorithms
{
Ramp::Ramp(const float& initialValue)
    : target(initialValue),
      value(initialValue),
      targetReached(true)
{
}

void Ramp::setTarget(const float& target)
{
    if (!compareFloatClose(target, this->target, RAMP_EPSILON))
    {
        this->target = target;
        this->targetReached = false;
    }
}

void Ramp::update(float increment)
{
    increment = copysign(increment, target - value);
    float targetValueDifference = copysign(target - value, increment);
    value = fabs(targetValueDifference) > fabs(increment) ? value + increment : target;
    targetReached = compareFloatClose(value, target, RAMP_EPSILON);
}

const float& Ramp::getValue() const { return this->value; }

bool Ramp::isTargetReached() const { return targetReached; }

const float& Ramp::getTarget() const { return target; }

}  // namespace algorithms

}  // namespace aruwlib
