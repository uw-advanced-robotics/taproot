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

#ifndef AGITATOR_SUBSYSTEM_HPP_
#define AGITATOR_SUBSYSTEM_HPP_

#include "aruwlib/architecture/conditional_timer.hpp"
#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/control/subsystem.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/DJIMotorMock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

#include "aruwlib/control/setpoint/algorithms/setpoint_continuous_jam_checker.hpp"
#include "aruwlib/control/setpoint/interfaces/setpoint_subsystem.hpp"

#include "aruwsrc/algorithms/turret_pid.hpp"

#include "util_macros.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * Subsystem whose primary purpose is to encapsulate an agitator motor
 * that operates using a position controller. While this subsystem provides
 * direct support for agitator control, it is generic enough to be used in a
 * wide variety of senarios.
 */
class AgitatorSubsystem : public aruwlib::control::setpoint::SetpointSubsystem
{
public:
#if defined(TARGET_SOLDIER) || defined(TARGET_OLD_SOLDIER)
    // position PID terms
    // PID terms for soldier
    static constexpr float PID_17MM_P = 170000.0f;
    static constexpr float PID_HOPPER_P = 100000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 80.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

    static constexpr bool isAgitatorInverted = false;

    static constexpr float AGITATOR_JAMMING_DISTANCE = aruwlib::algorithms::PI / 5;

    // The motor that controls the hopper lid is an agitator_subsystem instance, so
    // I'm adding its constants here as well.
    static constexpr aruwlib::motor::MotorId HOPPER_COVER_MOTOR_ID = aruwlib::motor::MOTOR8;
    static constexpr aruwlib::can::CanBus HOPPER_COVER_MOTOR_CAN_BUS =
        aruwlib::can::CanBus::CAN_BUS1;

    static constexpr bool IS_HOPPER_COVER_INVERTED = false;

#elif defined(TARGET_SENTINEL)
    // position PID terms
    // PID terms for sentinel
    static constexpr float PID_17MM_P = 120000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 50.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

#elif defined(TARGET_HERO)
    // Hero's waterwheel constants
    static constexpr float PID_HERO_WATERWHEEL_P = 100000.0f;
    static constexpr float PID_HERO_WATERWHEEL_I = 0.0f;
    static constexpr float PID_HERO_WATERWHEEL_D = 10.0f;
    static constexpr float PID_HERO_WATERWHEEL_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_HERO_WATERWHEEL_MAX_OUT = 16000.0f;

    static constexpr aruwlib::motor::MotorId HERO_WATERWHEEL_MOTOR_ID = aruwlib::motor::MOTOR3;
    static constexpr aruwlib::can::CanBus HERO_WATERWHEEL_MOTOR_CAN_BUS =
        aruwlib::can::CanBus::CAN_BUS1;
    static constexpr bool HERO_WATERWHEEL_INVERTED = false;

    // PID terms for the hero kicker
    static constexpr float PID_HERO_KICKER_P = 50000.0f;
    static constexpr float PID_HERO_KICKER_I = 0.0f;
    static constexpr float PID_HERO_KICKER_D = 10.0f;
    static constexpr float PID_HERO_KICKER_MAX_ERR_SUM = 0.0f;
    // max out added by Tenzin since it wasn't here. This should
    // also be changed by someone who know's what they're doing!
    static constexpr float PID_HERO_KICKER_MAX_OUT = 16000.0f;

    // There are two kicker motors that drive the shaft.
    static constexpr aruwlib::motor::MotorId HERO_KICKER1_MOTOR_ID = aruwlib::motor::MOTOR7;
    static constexpr aruwlib::motor::MotorId HERO_KICKER2_MOTOR_ID = aruwlib::motor::MOTOR8;
    static constexpr aruwlib::can::CanBus HERO_KICKER1_MOTOR_CAN_BUS =
        aruwlib::can::CanBus::CAN_BUS1;
    static constexpr aruwlib::can::CanBus HERO_KICKER2_MOTOR_CAN_BUS =
        aruwlib::can::CanBus::CAN_BUS1;
    static constexpr bool HERO_KICKER_INVERTED = false;
#endif

    /**
     * Agitator gear ratios of different motors, for determining shaft rotation angle.
     */
    static constexpr float AGITATOR_GEAR_RATIO_M2006 = 36.0f;
    static constexpr float AGITATOR_GEAR_RATIO_GM3508 = 19.0f;

    /**
     * The jamming constants. Agitator is considered jammed if difference between setpoint
     * and current angle is > `JAMMING_DISTANCE` radians for >= `JAMMING_TIME` ms;
     */
    static constexpr float JAMMING_DISTANCE = 1.0f;
    static constexpr uint32_t JAMMING_TIME = 250;

    /**
     * Construct an agitator with the passed in PID parameters, gear ratio, and motor-specific
     * identifiers.
     */
    AgitatorSubsystem(
        aruwlib::Drivers* drivers,
        float kp,
        float ki,
        float kd,
        float maxIAccum,
        float maxOutput,
        float agitatorGearRatio,
        aruwlib::motor::MotorId agitatorMotorId,
        aruwlib::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted,
        bool jamLogicEnabled = true,
        float jammingDistance = JAMMING_DISTANCE,
        uint32_t jammingTime = JAMMING_TIME);

    void initialize() override;

    void refresh() override;

    /**
     * @return The angle set in `setSetpoint`.
     */
    mockable inline float getSetpoint() const override { return desiredAgitatorAngle; }

    /**
     * Sets desired angle in radians of the agitator motor, relative to where the agitator
     * has been initialized.
     *
     * @param[in] newAngle The desired angle.
     */
    mockable inline void setSetpoint(float newAngle) override { desiredAgitatorAngle = newAngle; }

    /**
     * @return The calibrated agitator angle, in radians. If the agitator is uncalibrated, 0
     *      radians is returned.
     */
    mockable float getCurrentValue() const override;

    /**
     * Attempts to calibrate the agitator at the current position, such that
     * `getCurrentValue` will return 0 radians at this position.
     *
     * @return `true` if the agitator has been successfully calibrated, `false`
     *      otherwise.
     */
    mockable bool calibrateHere() override;

    /**
     * @return `true` if the agitator unjam timer has expired, signaling that the agitator
     *      has jammed, `false` otherwise.
     */
    mockable bool isJammed() override { return jamLogicEnabled && subsystemJamStatus; }

    /**
     * Clear the jam status of the subsystem, indicating that it has been unjammed.
     */
    void clearJam() override { subsystemJamStatus = false; }

    /**
     * @return `true` if the agitator has been calibrated (`calibrateHere` has been
     *      called and the agitator motor is online).
     */
    mockable inline bool isCalibrated() override { return agitatorIsCalibrated; }

    /**
     * @return `true` if the agitator motor is online (i.e.: is connected)
     */
    mockable inline bool isOnline() override { return agitatorMotor.isMotorOnline(); }

    /**
     * @return The velocity of the agitator in units of degrees per second.
     */
    mockable inline float getVelocity() override
    {
        return 6.0f * static_cast<float>(agitatorMotor.getShaftRPM()) / gearRatio;
    }

    void runHardwareTests() override;

    void onHardwareTestStart() override;

    mockable const char* getName() override { return "Agitator"; }

protected:
    /**
     * Whether or not the agitator has been calibrated yet. You should calibrate the agitator
     * before using it.
     */
    bool agitatorIsCalibrated = false;

    void agitatorRunPositionPid();

private:
    /**
     * PID controller for running postiion PID on unwrapped agitator angle (in radians).
     */
    aruwsrc::algorithms::TurretPid agitatorPositionPid;

    /**
     * The object that runs jam detection.
     */
    aruwlib::control::setpoint::SetpointContinuousJamChecker jamChecker;

    /**
     * The user desired angle, measured in radians.
     * The agitator uses unwrapped angle.
     */
    float desiredAgitatorAngle = 0.0f;

    /**
     * You can calibrate the agitator, which will set the current agitator angle to zero radians.
     */
    float agitatorCalibratedZeroAngle = 0.0f;

    /**
     * Motor gear ratio, so we use shaft angle rather than encoder angle.
     */
    float gearRatio;

    /**
     * Stores the jam state of the subsystem
     */
    bool subsystemJamStatus = false;

    /**
     * A flag which determines whether or not jamming detection is enabled.
     * `true` means enabled, `false` means disabled.
     * Detailed effect: When `false`, isJammed() always return false.
     */
    bool jamLogicEnabled;

    /**
     * Get the raw angle of the shaft from the motor
     */
    float getUncalibratedAgitatorAngle() const;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    aruwlib::mock::DjiMotorMock agitatorMotor;

private:
#else
    aruwlib::motor::DjiMotor agitatorMotor;
#endif
};  // class AgitatorSubsystem

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_SUBSYSTEM_HPP_
