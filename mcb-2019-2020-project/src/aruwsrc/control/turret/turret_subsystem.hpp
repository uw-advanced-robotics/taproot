#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/algorithms/linear_interpolation.hpp>

namespace aruwsrc
{

namespace turret
{

class TurretSubsystem : public aruwlib::control::Subsystem {
 public:
    static constexpr float TURRET_START_ANGLE = 90.0f;
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 20.0f;

    TurretSubsystem();

    void refresh() override;

    bool isTurretOnline() const;

    int32_t getYawVelocity() const;
    int32_t getPitchVelocity() const;

    float getYawAngleFromCenter() const;
    float getPitchAngleFromCenter() const;

    const aruwlib::algorithms::ContiguousFloat& getYawAngle() const;
    const aruwlib::algorithms::ContiguousFloat& getPitchAngle() const;

    void setYawMotorOutput(float out);
    void setPitchMotorOutput(float out);

    /**
     * Calculates a yaw output that uses the desired chassis rotation as a feed forward gain.
     * 
     * The chassis rotation is given in desired wheel rpm.
     */
    float yawFeedForwardCalculation(float desiredChassisRotation);

    /**
     * Set a target angle in chassis frame, the angle is accordingly limited.
     * Note that since there is no controller in this subsystem, this target
     * angle merely acts as a safe way to set angles when using a position controller.
     */
    void setYawTarget(float target);
    void setPitchTarget(float target);

    float getYawTarget() const;
    float getPitchTarget() const;

    void updateCurrentTurretAngles();

 private:
    const uint16_t YAW_START_ENCODER_POSITION = 8160;
    const uint16_t PITCH_START_ENCODER_POSITION = 4100;

    static constexpr float FEED_FORWARD_KP = 2.0f;
    static constexpr float FEED_FORWARD_SIN_GAIN = 2.0f;
    static constexpr float FEED_FORWARD_KD = 1.0f;
    static constexpr float FEED_FORWARD_MAX_OUTPUT = 20000.0f;
    static constexpr float FEED_FORWARD_DERIVATIVE_LOW_PASS = 0.8f;

    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static const aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static const aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

    uint32_t prevUpdateCounterChassisRotateDerivative = 0;
    aruwlib::algorithms::LinearInterpolation chassisRotateDerivativeInterpolation;
    float feedforwardChassisRotateDerivative = 0.0f;
    float feedforwardPrevChassisRotationDesired = 0.0f;

    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;

    aruwlib::algorithms::ContiguousFloat currPitchAngle;
    aruwlib::algorithms::ContiguousFloat currYawAngle;

    aruwlib::algorithms::ContiguousFloat yawTarget;
    aruwlib::algorithms::ContiguousFloat pitchTarget;

    void updateCurrentYawAngle();
    void updateCurrentPitchAngle();

    int32_t getVelocity(const aruwlib::motor::DjiMotor &motor) const;
};

}  // namespace turret

}  // namespace aruwsrc

#endif
