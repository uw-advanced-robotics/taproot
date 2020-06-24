#ifndef __SUBSYSTEM_SENTINEL_DRIVE_HPP__
#define __SUBSYSTEM_SENTINEL_DRIVE_HPP__

#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/command_scheduler.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <modm/math/filter/pid.hpp>

#include "robot_type.hpp"

namespace aruwsrc
{
namespace control
{
class SentinelDriveSubsystem : public aruwlib::control::Subsystem
{
public:
    static constexpr float MAX_POWER_CONSUMPTION = 30.0f;
    static constexpr float MAX_ENERGY_BUFFER = 200.0f;

    // length of the rail we own, in mm
    // the competition rail length is actually 4650mm
    static constexpr float RAIL_LENGTH = 1900;

    SentinelDriveSubsystem(
        aruwlib::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        aruwlib::motor::MotorId rightMotorId = RIGHT_MOTOR_ID)
        : leftWheel(leftMotorId, CAN_BUS_MOTORS, false, "left sentinel drive motor"),
          rightWheel(rightMotorId, CAN_BUS_MOTORS, false, "right sentinel drive motor"),
          velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
          velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
          desiredRpm(0)
    {
    }

    void initialize() override;

    /**
     * Returns absolute position of the sentinel, relative to the left end of the rail (when rail
     * is viewed from the front)
     */
    float absolutePosition();

    void setDesiredRpm(float desRpm);

    void refresh() override;

private:
    static constexpr aruwlib::motor::MotorId LEFT_MOTOR_ID = aruwlib::motor::MOTOR6;
    static constexpr aruwlib::motor::MotorId RIGHT_MOTOR_ID = aruwlib::motor::MOTOR5;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    const aruwlib::gpio::Digital::InputPin leftLimitSwitch = aruwlib::gpio::Digital::InputPin::A;
    const aruwlib::gpio::Digital::InputPin rightLimitSwitch = aruwlib::gpio::Digital::InputPin::B;

    const float PID_P = 5.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.1f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    // radius of the wheel in mm
    static constexpr float WHEEL_RADIUS = 35.0f;
    static constexpr float GEAR_RATIO = 19.0f;

    aruwlib::motor::DjiMotor leftWheel;

    aruwlib::motor::DjiMotor rightWheel;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredRpm;
    float leftZeroRailOffset = 0;
    float rightZeroRailOffset = 0;

    void resetOffsetFromLimitSwitch();

    float distanceFromEncoder(aruwlib::motor::DjiMotor* motor);
};

}  // namespace control

}  // namespace aruwsrc

#endif
