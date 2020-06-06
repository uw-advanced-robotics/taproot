#ifndef PWM_HPP_
#define PWM_HPP_

#include <cstdint>

namespace aruwlib
{
namespace gpio
{
/**
 * PWM input pins are pins S, T, U, and V (board pins PAO, PA1, PA2, PA3)
 * To write a PWM frequency to a pin call `write` and pass the function a
 * value (W - Z) from the analog outPin enum and a PWM duty from 0.0-1.0
 * (where 1 is all HIGH and 0 is all LOW). To set the duty for all pins
 * call the `writeAll` function with only the duty.
 */
class Pwm
{
public:
    Pwm() = default;
    Pwm(const Pwm &) = delete;
    Pwm &operator=(const Pwm &) = default;

    /**
     * PWM pins whose name corresponds to the names defined on the
     * RoboMaster type A board.
     */
    enum Pin
    {
        W = 1,
        X,
        Y,
        Z
    };

    void init();

    /**
     * Sets all Timer channels to the same duty.
     *
     * @param[in] duty the duty cycle to be set. If the duty is outside of the range
     *      of [0, 1] the duty is limited to within the range.
     */
    void writeAll(float duty);

    /**
     * Sets the PWM duty for a specified pin.
     *
     * @param [in] duty the duty cycle to be set. If the duty is outside of the range
     *      of [0, 1] the duty is limited to within the range.
     * @param[in] pin the PWM pin to be set.
     */
    void write(float duty, Pwm::Pin pin);
};  // class Pwm

}  // namespace gpio

}  // namespace aruwlib

#endif  // PWM_HPP_
