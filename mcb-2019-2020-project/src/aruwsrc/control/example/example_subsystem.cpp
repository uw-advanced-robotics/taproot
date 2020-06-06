#include "example_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
const aruwlib::motor::MotorId ExampleSubsystem::LEFT_MOTOR_ID = aruwlib::motor::MOTOR2;
const aruwlib::motor::MotorId ExampleSubsystem::RIGHT_MOTOR_ID = aruwlib::motor::MOTOR1;

void ExampleSubsystem::setDesiredRpm(float desRpm) { desiredRpm = desRpm; }

void ExampleSubsystem::refresh()
{
    updateMotorRpmPid(&velocityPidLeftWheel, &leftWheel, desiredRpm);
    updateMotorRpmPid(&velocityPidRightWheel, &rightWheel, desiredRpm);
}

void ExampleSubsystem::updateMotorRpmPid(
    modm::Pid<float>* pid,
    aruwlib::motor::DjiMotor* motor,
    float desiredRpm)
{
    pid->update(desiredRpm - motor->getShaftRPM());
    motor->setDesiredOutput(static_cast<int32_t>(pid->getValue()));
}
}  // namespace control

}  // namespace aruwsrc
