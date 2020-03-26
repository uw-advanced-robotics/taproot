#ifndef __FRICTION_WHEEL_SUBSYSTEM_HPP__
#define __FRICTION_WHEEL_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/ramp.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace launcher
{

class FrictionWheelSubsystem : public Subsystem
{
 public:
    FrictionWheelSubsystem(
        aruwlib::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        aruwlib::motor::MotorId rightMotorId = RIGHT_MOTOR_ID)
        : leftWheel(leftMotorId, CAN_BUS_MOTORS, true, "left example motor"),
        rightWheel(rightMotorId, CAN_BUS_MOTORS, false, "right example motor"),
        velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        desiredRpmRamp(0)
    {}

    void setDesiredRpm(float desRpm);

    void refresh();

 private:
    static constexpr aruwlib::motor::MotorId LEFT_MOTOR_ID = aruwlib::motor::MOTOR2;
    static constexpr aruwlib::motor::MotorId RIGHT_MOTOR_ID = aruwlib::motor::MOTOR1;
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    // speed of ramp when you set a new desired ramp speed [rpm / ms]
    static constexpr float FRICTION_WHEEL_RAMP_SPEED = 1.0f;

    static constexpr float PID_P = 30.0f;
    static constexpr float PID_I = 0.0f;
    static constexpr float PID_D = 5.0f;
    static constexpr float PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 16000.0f;

    aruwlib::motor::DjiMotor leftWheel;

    aruwlib::motor::DjiMotor rightWheel;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    aruwlib::algorithms::Ramp desiredRpmRamp;

    uint32_t prevTime = 0;
};

}  // namespace launcher

}  // namespace aruwsrc

#endif
