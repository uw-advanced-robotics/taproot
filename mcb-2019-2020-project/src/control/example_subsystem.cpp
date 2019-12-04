#include "src/control/example_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    const aruwlib::motor::MotorId ExampleSubsystem::LEFT_MOTOR_ID = aruwlib::motor::MOTOR4;
    const aruwlib::motor::MotorId ExampleSubsystem::RIGHT_MOTOR_ID = aruwlib::motor::MOTOR5;

    void ExampleSubsystem::setDesiredRpm(float desRpm)
    {
        desiredRpm = desRpm;
    }

    void ExampleSubsystem::refresh()
    {
        updateMotorRpmPid(
            &velocityPidLeftWheel,
            &leftWheel, desiredRpm
        );
        updateMotorRpmPid(
            &velocityPidRightWheel,
            &rightWheel, desiredRpm
        );
    }

    void ExampleSubsystem::updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* motor,
        float desiredRpm
    ) {
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }
}  // namespace control

}  // namespace aruwsrc
