/* 
   Hopper subsystem is made of a servo that will spin
   to two certain angles determined by the user as 
   the open position and the close position
*/

#ifndef __OPEN_HOPPER_SUBSYSTEM__
#define __OPEN_HOPPER_SUBSYSTEM__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/servo.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class HopperSubsystem : public Subsystem
{
 public:
    static constexpr float SOLDIER_HOPPER_OPEN_PWM = 0.21f;
    static constexpr float SOLDIER_HOPPER_CLOSE_PWM = 0.11f;
    static constexpr float SOLDIER_PWM_RAMP_SPEED = 0.001f;

    /* 
     * constructor
     * @param currPort the pin that the servo is connected to
     * @param open     the angle defined as open; a PWM value
     *                 (between 0 and 1)
     * @param close    the angle defined as close; a PWM value
     *                 (between 0 and 1)  
     * @param pwmRampSpeed   determines the speed of servo operation;
     *                 a PWM value (between 0 and 1)   
     */
    HopperSubsystem(aruwlib::gpio::Pwm::Pin currPort,
                    float open,
                    float close,
                    float pwmRampSpeed) :
                    hopper(currPort, open, close, pwmRampSpeed)
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

    void refresh();

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
