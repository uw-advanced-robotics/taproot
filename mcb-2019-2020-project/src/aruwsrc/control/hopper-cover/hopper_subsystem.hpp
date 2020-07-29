/*
   Hopper subsystem is made of a servo that will spin
   to two certain angles determined by the user as
   the open position and the close position
*/

#ifndef __OPEN_HOPPER_SUBSYSTEM__
#define __OPEN_HOPPER_SUBSYSTEM__

#include <aruwlib/control/command_scheduler.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/servo.hpp>
#include <modm/math/filter/pid.hpp>

namespace aruwsrc
{
namespace control
{
class HopperSubsystem : public aruwlib::control::Subsystem
{
public:
#if defined(TARGET_SOLDIER)
    static constexpr float SOLDIER_HOPPER_OPEN_PWM = 0.21f;
    static constexpr float SOLDIER_HOPPER_CLOSE_PWM = 0.11f;
    static constexpr float SOLDIER_PWM_RAMP_SPEED = 0.001f;
#elif defined(TARGET_OLD_SOLDIER)
    static constexpr float OLD_SOLDIER_HOPPER_OPEN_PWM = 0.21f;
    static constexpr float OLD_SOLDIER_HOPPER_CLOSE_PWM = 0.11f;
    static constexpr float OLD_SOLDIER_PWM_RAMP_SPEED = 0.001f;
#endif

    /*
     * constructor
     * @param pwmPin the pin that the servo is connected to
     * @param open     the angle defined as open; a PWM value
     *                 (between 0 and 1)
     * @param close    the angle defined as close; a PWM value
     *                 (between 0 and 1)
     * @param pwmRampSpeed   determines the speed of servo operation;
     *                 a PWM value (between 0 and 1)
     */
    HopperSubsystem(aruwlib::gpio::Pwm::Pin pwmPin, float open, float close, float pwmRampSpeed)
        : hopper(pwmPin, open, close, pwmRampSpeed)
    {
        hopper.setTargetPwm(close);
    }

    /*
     * set servo to the open angle
     */
    void setOpen();

    /*
     * set servo to the close angle
     */
    void setClose();

    void refresh() override;

private:
    aruwlib::motor::Servo hopper;

    /*
     * return the angle defined as open as a PWM value
     */
    float getOpenPWM();

    /*
     * return the angle defined as close as a PWM value
     */
    float getClosePWM();
};

}  // namespace control

}  // namespace aruwsrc

#endif
