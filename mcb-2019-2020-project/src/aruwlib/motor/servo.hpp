// servo initialization and movement class
// ensuring degree measurements are input and output is actually the servo movement.
#ifndef _SERVO_HPP_
#define _SERVO_HPP_

#include "aruwlib/communication/gpio/pwm.hpp"
#include "aruwlib/algorithms/ramp.hpp"

namespace aruwlib
{

namespace motor
{

class Servo {
 private:
    // used to change servo speed
    // see construtctor for detail
    aruwlib::algorithms::Ramp pwmOutputRamp;

    // max pwm the servo can handle, default is 1 but this is likely incorrect depending on
    // what type of servo you have
    float maxPWM = 1.0f;

    // min pwm the servo can handle, default is 0 but this is likely incorrect depending on
    // what type of servo you have
    float minPWM = 0.0f;

    // current pwm output
    float currentPWM = 0.0f;

    // desired speed of the ramp in pwm / ms
    float pwmRampSpeed = 0.0f;

    uint32_t prevTime = 0;

    // current port for output
    aruwlib::gpio::Pwm::Pin servoPin;

 public:
    // constructor
    // initializes class with PWMs; variable "pwmRampSpeed" is used for constructing pwmOutputRamp
    // the greater "pwmRampSpeed" is, the faster the servo will rotate
    Servo(aruwlib::gpio::Pwm::Pin currPort,
        const float maximumPWM, float minimumPWM, float pwmRampSpeed);

    // sets the pwmOutputRamp object to the desired RPM.
    // do not repeatedly call
    void setTargetPwm(float PWM);

    // update the pwmOutputRamp object
    // in the process, sets the output pwm to the updated ramp pwm value
    void updateSendPwmRamp();

    // gets the pwm output to said float value.
    float getPWM();

    // gets the minimum pwm output
    float getMinPWM() const;

    // gets the maximum pwm output
    float getMaxPWM() const;

    // returns true if the ramp has met the desired pwm value (set with setTargetPwm)
    // use this to estimate when a servo movement is complete
    bool isRampTargetMet() const;
};

}  // namespace motor

}  // namespace aruwlib

#endif  // _SERVO_HPP_
