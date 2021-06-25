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

#include "aruwlib/algorithms/contiguous_float.hpp"
#include "aruwlib/algorithms/linear_interpolation.hpp"
#include "aruwlib/control/turret/i_turret_subsystem.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/DJIMotorMock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

#include "util_macros.hpp"

namespace aruwsrc
{
namespace turret
{
/**
 * Stores software necessary for interacting with two gimbals that control the pitch and
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 */
class TurretSubsystem : public aruwlib::control::turret::ITurretSubsystem
{
public:
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
    static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
    static constexpr aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;

#if defined(TARGET_SOLDIER)
    static constexpr float TURRET_START_ANGLE = 90.0f;
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 30.0f;
#elif defined(TARGET_HERO)
    static constexpr float TURRET_START_ANGLE = 90.0f;
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 70.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 70.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = 65.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = 104.0f;
#else
    static constexpr float TURRET_START_ANGLE = 90.0f;
    static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
    static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
    static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
    static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 30.0f;
#endif

    /**
     * Constructs a TurretSubsystem.
     *
     * @param[in] drivers Pointer to a drivers singleton object
     * @param[in] limitYaw `true` if the yaw should be limited between `TURRET_YAW_MIN_ANGLE` and
     *      `TURRET_YAW_MAX_ANGLE` and `false` if the yaw should not be limited (if you have a slip
     *      ring).
     */
    explicit TurretSubsystem(aruwlib::Drivers* drivers, bool limitYaw = true);

    inline bool yawLimited() const { return limitYaw; }

    void initialize() override;

    void refresh() override;

    const char* getName() override { return "Turret"; }

    void onHardwareTestStart() override;

    /**
     * @return `true` if both pitch and yaw gimbals are connected.
     */
    inline bool isOnline() const override
    {
        return yawMotor.isMotorOnline() && pitchMotor.isMotorOnline();
    }

    /**
     * @return The wrapped yaw angle of the actual yaw gimbal, in degrees
     */
    inline const aruwlib::algorithms::ContiguousFloat& getCurrentYawValue() const override
    {
        return currYawAngle;
    }

    /**
     * @return The wrapped pitch angle of the actual pitch gimbal, in degrees.
     */
    inline const aruwlib::algorithms::ContiguousFloat& getCurrentPitchValue() const override
    {
        return currPitchAngle;
    }

    /**
     * @return The yaw target as set by the user in `setYawSetpoint`.
     */
    inline float getYawSetpoint() const override { return yawTarget.getValue(); }

    /**
     * @return The pitch target as set by the user in `setPitchSetpoint`.
     */
    inline float getPitchSetpoint() const override { return pitchTarget.getValue(); }

    /**
     * @return The velocity, in degrees / second, of the turret's pitch yaw
     */
    inline float getYawVelocity() const override { return getVelocity(yawMotor); }

    /**
     * @return The velocity, in degrees / second, of the turret's pitch motor
     */
    inline float getPitchVelocity() const override { return getVelocity(pitchMotor); }

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
     * @return An angle between [-180, 180] that is the angle difference of the yaw gimbal
     *      from center (90 degrees), in degrees.
     */
    mockable float getYawAngleFromCenter() const;

    /**
     * @return An angle between [-180, 180] that is the angle difference of the pitch gimbal
     *      from center (90 degrees), in degrees.
     */
    mockable float getPitchAngleFromCenter() const;

    /**
     * Attempts to set desired yaw output to the passed in value. If the turret is out of
     * bounds, the output is limited.
     *
     * @param[in] out The desired yaw output, limited to `[-30000, 30000]`.
     */
    mockable void setYawMotorOutput(float out);

    /**
     * Attempts to set desired pitch output to the passed in value. If the turret is out of
     * bounds, the output is limited.
     *
     * @param[in] out The desired pitch output, limited to `[-30000, 30000]`.
     */
    mockable void setPitchMotorOutput(float out);

    /**
     * Calculates a yaw output that uses the desired chassis rotation as a feed forward gain.
     *
     * @param[in] desiredChassisRotation The chassis rotation in RPM (before gearing).
     */
    mockable float yawFeedForwardCalculation(float desiredChassisRotation);

    /**
     * Reads the raw pitch and yaw angles and updates the wrapped versions of
     * these angles.
     */
    mockable void updateCurrentTurretAngles();

private:
#if defined(TARGET_SOLDIER)
    static constexpr uint16_t YAW_START_ENCODER_POSITION = 6821;
    static constexpr uint16_t PITCH_START_ENCODER_POSITION = 4100;
#elif defined(TARGET_HERO)
    static constexpr uint16_t YAW_START_ENCODER_POSITION = 3000;
    static constexpr uint16_t PITCH_START_ENCODER_POSITION = 1418;
#else
    static constexpr uint16_t YAW_START_ENCODER_POSITION = 0;
    static constexpr uint16_t PITCH_START_ENCODER_POSITION = 4100;
#endif

    static constexpr float FEED_FORWARD_KP = 11800.0f;
    static constexpr float FEED_FORWARD_MAX_OUTPUT = 20000.0f;

    uint32_t prevUpdateCounterChassisRotateDerivative = 0;
    aruwlib::algorithms::LinearInterpolation chassisRotateDerivativeInterpolation;
    float feedforwardChassisRotateDerivative = 0.0f;
    float feedforwardPrevChassisRotationDesired = 0.0f;

    aruwlib::algorithms::ContiguousFloat currPitchAngle;
    aruwlib::algorithms::ContiguousFloat currYawAngle;

    aruwlib::algorithms::ContiguousFloat yawTarget;
    aruwlib::algorithms::ContiguousFloat pitchTarget;

    bool limitYaw;

    void updateCurrentYawAngle();
    void updateCurrentPitchAngle();

    /**
     * @return velocity of 6020 motor, in degrees / sec
     */
    static inline float getVelocity(const aruwlib::motor::DjiMotor& motor)
    {
        return 360 / 60 * motor.getShaftRPM();
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    aruwlib::mock::DjiMotorMock pitchMotor;
    aruwlib::mock::DjiMotorMock yawMotor;

private:
#else
    aruwlib::motor::DjiMotor pitchMotor;
    aruwlib::motor::DjiMotor yawMotor;
#endif

};  // class TurretSubsystem

}  // namespace turret

}  // namespace aruwsrc

#endif  // TURRET_SUBSYSTEM_HPP_
