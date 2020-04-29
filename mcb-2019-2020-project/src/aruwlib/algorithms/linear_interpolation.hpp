#ifndef __LINEAR_INTERPOLATION_HPP__
#define __LINEAR_INTERPOLATION_HPP__

#include <cstdint>

#include <modm/architecture/interface/clock.hpp>

namespace aruwlib
{

namespace algorithms
{

class LinearInterpolation
{
 public:
    LinearInterpolation();

    // only call this when you receive a new value (use remote rx counter for example)
    void update(float newValue);

    // use modm::Clock::now().getTime()
    float getInterpolatedValue(uint32_t currTime);

 private:
    uint32_t lastUpdateCallTime;
    float previousValue;
    float slope;
};

}  // namespace algorithms

}  // namespace aruwlib

#endif