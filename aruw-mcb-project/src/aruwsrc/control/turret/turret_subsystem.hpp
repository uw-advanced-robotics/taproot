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

#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/algorithms/linear_interpolation.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <modm/math/filter/pid.hpp>

namespace aruwsrc
{
namespace turret
{
/**
 * Stores software necessary for interacting with two gimbals that control the pitch and
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 */
class TurretSubsystem : public aruwlib::control::Subsystem
{
public:
    static constexpr float TURRET_START_ANGLE = 90.0f;
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 20.0f;

    explicit TurretSubsystem(aruwlib::Drivers* drivers);

    void initialize() override;

    void refresh() override;

    /**
     * @return `true` if both pitch and yaw gimbals are connected.
     */
    bool isTurretOnline() const;

    int32_t getYawVelocity() const;
    int32_t getPitchVelocity() const;

    /**
     * @return An angle between [-180, 180] that is the angle difference of the yaw gimbal
     *      from center (90 degrees).
     */
    float getYawAngleFromCenter() const;
    /**
     * @return An angle between [-180, 180] that is the angle difference of the pitch gimbal
     *      from center (90 degrees).
     */
    float getPitchAngleFromCenter() const;

    /**
     * @return The wrapped yaw angle of the actual yaw gimbal.
     */
    const aruwlib::algorithms::ContiguousFloat& getYawAngle() const;
    /**
     * @return The wrapped pitch angle of the actual pitch gimbal.
     */
    const aruwlib::algorithms::ContiguousFloat& getPitchAngle() const;

    /**
     * Attempts to set desired yaw output to the passed in value. If the turret is out of
     * bounds, the output is limited.
     *
     * @param[in] out The desired yaw output, limited to `[-30000, 30000]`.
     */
    void setYawMotorOutput(float out);
    /**
     * Attempts to set desired pitch output to the passed in value. If the turret is out of
     * bounds, the output is limited.
     *
     * @param[in] out The desired pitch output, limited to `[-30000, 30000]`.
     */
    void setPitchMotorOutput(float out);

    /**
     * Calculates a yaw output that uses the desired chassis rotation as a feed forward gain.
     *
     * @param[in] desiredChassisRotation The chassis rotation in RPM (before gearing).
     */
    float yawFeedForwardCalculation(float desiredChassisRotation);

    /**
     * Set a target angle in chassis frame, the angle is accordingly limited.
     * Note that since there is no controller in this subsystem, this target
     * angle merely acts as a safe way to set angles when using a position controller.
     */
    void setYawTarget(float target);
    /**
     * @see setYawTarget
     */
    void setPitchTarget(float target);

    /**
     * @return The yaw target as set by the user in `setYawTarget`.
     */
    float getYawTarget() const;
    /**
     * @return The pitch target as set by the user in `setPitchTarget`.
     */
    float getPitchTarget() const;

    /**
     * Reads the raw pitch and yaw angles and updates the wrapped versions of
     * these angles.
     */
    void updateCurrentTurretAngles();

    void runHardwareTests() override;

    const char* getName() override { return "Turret Subsystem"; }

private:
    static constexpr uint16_t YAW_START_ENCODER_POSITION = 8160;
    static constexpr uint16_t PITCH_START_ENCODER_POSITION = 4100;

    static constexpr float FEED_FORWARD_KP = 2.0f;
    static constexpr float FEED_FORWARD_SIN_GAIN = 2.0f;
    static constexpr float FEED_FORWARD_KD = 1.0f;
    static constexpr float FEED_FORWARD_MAX_OUTPUT = 20000.0f;
    static constexpr float FEED_FORWARD_DERIVATIVE_LOW_PASS = 0.8f;

    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static constexpr aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

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

    int32_t getVelocity(const aruwlib::motor::DjiMotor& motor) const;
};  // class TurretSubsystem

}  // namespace turret

}  // namespace aruwsrc

#endif  // TURRET_SUBSYSTEM_HPP_
