/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SENTINEL_TURRET_SUBSYSTEM_HPP_
#define SENTINEL_TURRET_SUBSYSTEM_HPP_

#include <modm/math/filter/pid.hpp>

#include "aruwlib/algorithms/contiguous_float.hpp"
#include "aruwlib/algorithms/linear_interpolation.hpp"
#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/control/turret/turret_subsystem_interface.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/dji_motor_mock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

namespace aruwsrc::control::turret
{
/**
 * Stores software necessary for interacting with two gimbals that control the pitch and
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 */
class DoublePitchTurretSubsystem : public aruwlib::control::turret::TurretSubsystemInterface
{
public:
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID_LEFT = aruwlib::motor::MOTOR8;
    static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID_RIGHT = aruwlib::motor::MOTOR5;
    static constexpr aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR6;

    static constexpr float TURRET_YAW_START_ANGLE = 90.0f;
    static constexpr float TURRET_YAW_MIN_ANGLE = 5.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = 175.0f;
    static constexpr float TURRET_PITCH_START_ANGLE = 62.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = 45.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = 90.0f;

    static constexpr float YAW_P = 2000.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 120.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 30.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 1300.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 80.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 30.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 0.75f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 0.5f;

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 0.0f;

    static constexpr uint16_t YAW_START_ENCODER_POSITION = 6704;
    static constexpr uint16_t PITCH_90DEG_ENCODER_POSITION_LEFT = 5835;
    static constexpr uint16_t PITCH_90DEG_ENCODER_POSITION_RIGHT = 3123;

    explicit DoublePitchTurretSubsystem(aruwlib::Drivers* drivers, bool limitYaw = true);

    void initialize() override;

    void refresh() override;

    /**
     * @return The yaw target as set by the user in `setYawSetpoint`.
     */
    inline float getYawSetpoint() const override { return yawTarget.getValue(); }

    /**
     * @return The pitch target as set by the user in `setPitchSetpoint`.
     */
    inline float getPitchSetpoint() const override { return pitchTarget.getValue(); }

    /**
     * Set a target angle in chassis frame, the angle is accordingly limited.
     * Note that since there is no controller in this subsystem, this target
     * angle merely acts as a safe way to set angles when using a position controller.
     */
    void setYawSetpoint(float target) override;

    /**
     * @see setYawSetpoint
     */
    void setPitchSetpoint(float target) override;

    /**
     * @return The wrapped yaw angle of the actual yaw gimbal.
     */
    const aruwlib::algorithms::ContiguousFloat& getCurrentYawValue() const override;

    /**
     * @see getCurrentYawValue.
     */
    const aruwlib::algorithms::ContiguousFloat& getCurrentPitchValue() const override;

    /**
     * @return `true` if both pitch and yaw gimbals are connected.
     */
    inline bool isOnline() const override
    {
        return yawMotor.isMotorOnline() && pitchMotorLeft.isMotorOnline() &&
               pitchMotorRight.isMotorOnline();
    }

    /**
     * @return the yaw velocity of the turret, in degrees / second
     */
    inline float getYawVelocity() const override { return getVelocity(yawMotor); }

    /**
     * @see getYawVelocity.
     */
    inline float getPitchVelocity() const override
    {
        return (getVelocity(pitchMotorLeft) + getVelocity(pitchMotorRight)) / 2.0f;
    }

    /**
     * @return An angle between [-180, 180] that is the angle difference of the yaw gimbal
     *      from center (90 degrees), in degrees.
     */
    float getYawAngleFromCenter() const override;

    /**
     * @return An angle between [-180, 180] that is the angle difference of the pitch gimbal
     *      from center (90 degrees), in degrees.
     */
    float getPitchAngleFromCenter() const override;

    const char* getName() override { return "Sentinel Turret"; }

private:
    aruwlib::algorithms::ContiguousFloat currLeftPitchAngle;
    aruwlib::algorithms::ContiguousFloat currRightPitchAngle;
    aruwlib::algorithms::ContiguousFloat currYawAngle;

    aruwlib::algorithms::ContiguousFloat yawTarget;
    aruwlib::algorithms::ContiguousFloat pitchTarget;

    aruwsrc::algorithms::SmoothPid yawMotorPid;
    aruwsrc::algorithms::SmoothPid leftPitchPid;
    aruwsrc::algorithms::SmoothPid rightPitchPid;

    uint32_t prevTime;

    bool limitYaw;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    aruwlib::mock::DjiMotorMock pitchMotorLeft;
    aruwlib::mock::DjiMotorMock pitchMotorRight;
    aruwlib::mock::DjiMotorMock yawMotor;

private:
#else
    aruwlib::motor::DjiMotor pitchMotorLeft;
    aruwlib::motor::DjiMotor pitchMotorRight;
    aruwlib::motor::DjiMotor yawMotor;
#endif

    /**
     * Reads the raw pitch and yaw angles and updates the wrapped versions of
     * these angles.
     */
    void updateCurrentTurretAngles();

    /**
     * Update the angle of the specified motor.
     *
     * @param[in] motor The motor whose angle to read.
     * @param[in] calibrationEncoderValue The encoder value associated with the calibration angle.
     * @param[in] calibrationAngle The angle that corresponds to the calibrationEncoderValue. So if
     *      the calibrationEncoderValue is 1000 and the calibrationAngle is 90, when the motor's
     *      encoder reads 1000, the angle will be 90 an all other motor angles will be associated on
     *      this relationship.
     * @param[out] turretAngle The wrapped angle to update.
     */
    static void updateTurretAngle(
        const aruwlib::motor::DjiMotor& motor,
        uint16_t calibrationEncoderValue,
        float calibrationAngle,
        aruwlib::algorithms::ContiguousFloat& turretAngle);

    /**
     * Runs the passed in motor's PID controller given the specified motor motor,
     * angle, and other controller information.
     *
     * @param[in] currAngle The current wrapped turret angle associated with the motor.
     * @param[in] setpoint The setpoint used to compare with the currAngle when determining the
     *      desired motor output.
     * @param[in] dt The delta time between the current and previous calls to refresh.
     * @param[in] errorBtwnMotors
     * @param[in] pitchGravityCompensation
     * @param[out] pidController The PID controller to use and update.
     * @param[out] motor The motor to set the output on.
     */
    static void runPositionPid(
        const aruwlib::algorithms::ContiguousFloat& currAngle,
        const aruwlib::algorithms::ContiguousFloat& setpoint,
        const uint32_t dt,
        const float errorBtwnMotors,
        const float pitchGravityCompensation,
        algorithms::SmoothPid& pidController,
        aruwlib::motor::DjiMotor& motor);

    /**
     * Attempts to set desired passed in motor output to the passed in motor. If the turret is out
     * of bounds, the output is limited.
     *
     * @param[in] out The desired output, limited to `[-30000, 30000]`.
     * @param[in] currAngle The current motor angle, in degrees
     * @param[in] motor Reference to the motor to set.
     */
    static void setMotorOutput(float out, aruwlib::motor::DjiMotor& motor);

    /**
     * @return the angular velocity of a motor, in degrees / second, of the passed in motor. 360
     *      degrees / (60 seconds / minute) * (shaftRPM in shaft rotation / minute).
     */
    static inline int32_t getVelocity(const aruwlib::motor::DjiMotor& motor)
    {
        return 360 / 60 * static_cast<int32_t>(motor.getShaftRPM());
    }
};  // class DoublePitchTurretSubsystem

}  // namespace aruwsrc::control::turret

#endif  // SENTINEL_TURRET_SUBSYSTEM_HPP_
