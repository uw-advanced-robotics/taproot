/**
 * This is part of aruw's library.
 * 
 * This is example code for running friction wheels. As you can
 * see, there is a generic update pid loop that is independent of
 * what command is given to the subsystem. Additionally, the
 * subsystem contains variables specific to the subsystem
 * (pid controllers, motors, etc). If a control loop is specific
 * to a command, it should NOT be in a subsystem. For example,
 * control code to pulse the friction wheels should be located
 * outside of this class because pulsing is a specific command.
 */

#ifndef __SUBSYSTEM_EXAMPLE_HPP__
#define __SUBSYSTEM_EXAMPLE_HPP__

#include <modm/math/filter/pid.hpp>
#include <aruwlib/control/command_scheduler.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>

namespace aruwsrc
{

namespace control
{

class ExampleSubsystem : public aruwlib::control::Subsystem
{
 public:
    ExampleSubsystem(
        aruwlib::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        aruwlib::motor::MotorId rightMotorId = RIGHT_MOTOR_ID)
        : leftWheel(leftMotorId, CAN_BUS_MOTORS, true, "left example motor"),
        rightWheel(rightMotorId, CAN_BUS_MOTORS, false, "right example motor"),
        velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        desiredRpm(0)
    {}

    void setDesiredRpm(float desRpm);

    void refresh() override;

 private:
    static const aruwlib::motor::MotorId LEFT_MOTOR_ID;
    static const aruwlib::motor::MotorId RIGHT_MOTOR_ID;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    const float PID_P = 5.0f;
    const float PID_I = 0.0f;
    const float PID_D = 1.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    aruwlib::motor::DjiMotor leftWheel;

    aruwlib::motor::DjiMotor rightWheel;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredRpm;

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm
    );
};

}  // namespace control

}  // namespace aruwsrc

#endif
