#include "linear_interpolation.hpp"

#include <aruwlib/architecture/clock.hpp>

namespace aruwlib
{

namespace algorithms
{
    LinearInterpolation::LinearInterpolation() :
        lastUpdateCallTime(0),
        previousValue(0.0f),
        slope(0.0f)
    {}

    void LinearInterpolation::update(float newValue)
    {
        uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
        slope = (newValue - previousValue) / (currTime - lastUpdateCallTime);
        previousValue = newValue;
        lastUpdateCallTime = currTime;
    }

float slope1 = 0;
    float LinearInterpolation::getInterpolatedValue(uint32_t currTime)
    {
        slope1 = slope;
        return slope * static_cast<float>(currTime - lastUpdateCallTime) + previousValue;
    }
}  // namespace algorithms

}  // namespace aruwlib
