#ifndef __RAMP_HPP__
#define __RAMP_HPP__

#include <stdint.h>

namespace aruwlib
{

namespace algorithms
{
/**
 * The output value is incremented or decremented at every call to update
 * until target has been reached.
 * 
 * This is very similar to modm's ramp except for one difference: rather than
 * setting the increment at the beginning, you set the increment each time,
 * which allows you to take into account systems where time increment is not
 * constant.
 */
class Ramp
{
 public:
    /**
     * \brief    Create a ramp generator
     * \param    initialValue    Starting value
     */
    explicit Ramp(const float& initialValue = 0.0f);

    void setTarget(const float& target);

    // pass in timestamp in case period is not constant
    void update(float increment);

    const float& getValue() const;

    bool isTargetReached() const;

    const float& getTarget() const;

 private:
    static constexpr float RAMP_EPSILON = 0.00000000001f;
    float target;
    float value;
    bool targetReached;
};

}  // namespace algorithms

}  // namespace aruwlib

#endif
