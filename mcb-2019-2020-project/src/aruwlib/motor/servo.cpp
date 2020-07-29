#include "servo.hpp"

#include "aruwlib/Drivers.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/errors/create_errors.hpp"

namespace aruwlib
{
namespace motor
{
Servo::Servo(aruwlib::gpio::Pwm::Pin pwmPin, float maximumPwm, float minimumPwm, float pwmRampSpeed)
    : pwmOutputRamp(0.0f),
      maxPwm(aruwlib::algorithms::limitVal<float>(maximumPwm, 0.0f, 1.0f)),
      minPwm(aruwlib::algorithms::limitVal<float>(minimumPwm, 0.0f, 1.0f)),
      pwmRampSpeed(pwmRampSpeed),
      prevTime(0),
      servoPin(pwmPin)
{
    if (maxPwm < minPwm)
    {
        minPwm = 0.0f;
        maxPwm = 1.0f;
        RAISE_ERROR(
            "min servo PWM > max servo PWM",
            errors::Location::SERVO,
            errors::ErrorType::INVALID_ADD);
    }
}

void Servo::setTargetPwm(float pwm)
{
    pwmOutputRamp.setTarget(aruwlib::algorithms::limitVal<float>(pwm, minPwm, maxPwm));
    prevTime = aruwlib::arch::clock::getTimeMilliseconds();
}

void Servo::updateSendPwmRamp()
{
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    pwmOutputRamp.update(pwmRampSpeed * (currTime - prevTime));
    prevTime = currTime;
    currentPwm = pwmOutputRamp.getValue();
    Drivers::pwm.write(pwmOutputRamp.getValue(), servoPin);
}

float Servo::getPWM() const { return currentPwm; }

float Servo::getMinPWM() const { return minPwm; }

float Servo::getMaxPWM() const { return maxPwm; }

bool Servo::isRampTargetMet() const { return pwmOutputRamp.isTargetReached(); }

}  // namespace motor

}  // namespace aruwlib
