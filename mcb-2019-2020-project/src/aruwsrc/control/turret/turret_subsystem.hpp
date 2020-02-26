#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/contiguous_float.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem : public Subsystem {
 public:
    TurretSubsystem();

    void refresh();

    bool isTurretOnline() const;

    int32_t getYawVelocity() const;
    int32_t getPitchVelocity() const;

    float getYawAngleFromCenter() const;
    float getPitchAngleFromCenter() const;

    const aruwlib::algorithms::ContiguousFloat& getYawAngle() const;
    const aruwlib::algorithms::ContiguousFloat& getPitchAngle() const;

    void setYawMotorOutput(float out);
    void setPitchMotorOutput(float out);

    float getRemoteXMovement() const;
    float getRemoteYMovement() const;

    int16_t getMouseXMovement() const;
    int16_t getMouseYMovement() const;

 private:
    const int TURRET_START_ANGLE = 90.0f;
    const int TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90;
    const int TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90;
    const int TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 15;
    const int TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 20;

    const uint16_t YAW_START_ENCODER_POSITION = 8160;
    const uint16_t PITCH_START_ENCODER_POSITION = 4780;

    const float REMOTE_INPUT_SCALER = 10000;
    const float KEYBOARD_INPUT_SCALAR = 50;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

    aruwlib::algorithms::ContiguousFloat currPitchAngle;
    aruwlib::algorithms::ContiguousFloat currYawAngle;

    void updateCurrentTurretAngles();

    int32_t getVelocity(const aruwlib::motor::DjiMotor &motor) const;
};

}  // namespace control

}  // namespace aruwsrc

#endif
