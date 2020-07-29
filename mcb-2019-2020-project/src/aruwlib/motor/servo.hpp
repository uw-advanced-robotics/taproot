#ifndef SERVO_HPP_
#define SERVO_HPP_

#include "aruwlib/algorithms/ramp.hpp"
#include "aruwlib/communication/gpio/pwm.hpp"

namespace aruwlib
{
namespace motor
{
/**
 * This class wraps around the PWM class to provide utilities for controlling a servo.
 * In particular, this class limits some target PWM to a min and max PWM value and uses
 * ramping to control the speed of the servo.
 */
class Servo
{
public:
    /**
     * Initializes the PWM bounds and associates the Servo with some PWM pin.
     *
     * @note `maximumPwm` and `minimumPwm` are limited to between [0, 1]. Also if
     *      `maximumPwm` < `minimumPwm`, an error is thrown and [minPwm, maxPwm]
     *      is set to [0, 1].
     *
     * @param[in] pwmPin The pin to attach the Servo class with.
     * @param[in] maximumPwm The maximum allowable PWM output. This is limited between 0 and 1.
     * @param[in] minimumPwm The minimum allowable PWM output. This is limited between 0 and 1.
     * @param[in] pwmRampSpeed The speed in PWM percent per millisecond.
     */
    Servo(aruwlib::gpio::Pwm::Pin pwmPin, float maximumPwm, float minimumPwm, float pwmRampSpeed);

    /**
     * Limits `pwmOutputRamp` to `minPwm` and `maxPwm`, then sets ramp output
     * to the limited value. Do not repeatedly call (i.e. only call in a `Command`'s
     * `initialize` function, for example).
     */
    void setTargetPwm(float pwm);

    /**
     * Updates the `pwmOutputRamp` object and then sets the output PWM to the updated
     * ramp value.
     */
    void updateSendPwmRamp();

    /**
     * @return The current PWM output to the servo.
     */
    float getPWM() const;

    /**
     * @return The minimum PWM output (as a duty cycle).
     */
    float getMinPWM() const;

    /**
     * @return The maximum PWM output (as a duty cycle).
     */
    float getMaxPWM() const;

    /**
     * @return `true` if the ramp has met the desired PWM value (set with `setTargetPwm`).
     *      Use this to estimate when a servo movement is complete.
     */
    bool isRampTargetMet() const;

private:
    ///< Used to change servo speed. See construtctor for detail.
    aruwlib::algorithms::Ramp pwmOutputRamp;

    ///< The max PWM the servo can handle.
    float maxPwm;

    ///< The min PWM the servo can handle.
    float minPwm;

    ///< Current PWM output.
    float currentPwm;

    ///< Desired speed of the ramp in PWM / ms
    float pwmRampSpeed;

    ///< Used to calculate the ramp dt.
    uint32_t prevTime = 0;

    ///< The PWM pin that the servo is attached to.
    aruwlib::gpio::Pwm::Pin servoPin;
};  // class Servo

}  // namespace motor

}  // namespace aruwlib

#endif  // SERVO_HPP_
