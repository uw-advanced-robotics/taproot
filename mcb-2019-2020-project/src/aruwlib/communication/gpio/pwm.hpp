/*
 * PWM input pins are pins S, T, U, and V (board pins PAO, PA1, PA2, PA3)
 * To write a PWM frequency to a pin call Write and pass the 
 * function a value (W - Z) from the analog outPin enum and a PWM duty from 
 * 0.0-1.0 (where 1 is all HIGH and 0 is all LOW). To set the duty for all pins
 * call the Write function with only the duty.
 * 
 */

#ifndef PWM_HPP
#define PWM_HPP

namespace aruwlib
{

namespace gpio
{

class Pwm {
 private:
    #define PWM_RESOLUTION TIM8->ARR
 public:
    // PWM pins
    enum Pin
    {
        W = 1, X, Y, Z
    };

    static void init();

    // Sets all Timer channels to the same duty
    static void WriteAll(double duty);

    // Sets the PWM duty for a specified pin
    static void Write(double duty, Pwm::Pin pin);
};

}  // namespace gpio

}  // namespace aruwlib

#endif
