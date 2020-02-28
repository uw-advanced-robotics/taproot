#include "servo.hpp"
#include "rm-dev-board-a/board.hpp"

namespace aruwlib
{

namespace motor
{

Servo::Servo(aruwlib::gpio::Pwm::Pin currPort, float maximumPWM, float minimumPWM,
                                               float pwmRampSpeed) :
                                               pwmOutputRamp(0.0f),
                                               maxPWM(maximumPWM),
                                               minPWM(minimumPWM),
                                               pwmRampSpeed(pwmRampSpeed),
                                               prevTime(0),
                                               servoPin(currPort) {}

// giving the servo a specific PWM value
void Servo::setTargetPwm(float PWM) {
    pwmOutputRamp.setTarget(PWM);
    prevTime = modm::Clock::now().getTime();
}

// update the pwmOutputRamp object from loop in main
void Servo::updateSendPwmRamp(){
    uint32_t currTime = modm::Clock::now().getTime();
    pwmOutputRamp.update(pwmRampSpeed * (currTime - prevTime));
    prevTime = currTime;
    currentPWM = pwmOutputRamp.getValue();
    aruwlib::gpio::Pwm::Write(pwmOutputRamp.getValue(), servoPin);
}

// gets the current PWM output value.
float Servo::getPWM(){
    return currentPWM;
}

// gets the minimum pwm output
float Servo::getMinPWM() const {
    return minPWM;
}

// gets the maximum pwm output
float Servo::getMaxPWM() const {
    return maxPWM;
}

// returns true if the ramp has met the desired pwm value (set with setTargetPwm)
// use this to estimate when a servo movement is complete
bool Servo::isRampTargetMet() const {
    return pwmOutputRamp.isTargetReached();
}

}  // namespace motor

}  // namespace aruwlib
