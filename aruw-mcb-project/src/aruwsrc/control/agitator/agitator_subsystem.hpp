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

#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <modm/math/filter/pid.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"

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
class AgitatorSubsystem : public aruwlib::control::Subsystem
{
public:
#if defined(TARGET_SOLDIER) || defined(TARGET_OLD_SOLDIER)
    // position PID terms
    // PID terms for soldier
    static constexpr float PID_17MM_P = 170000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 80.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

    static constexpr bool isAgitatorInverted = false;

#elif defined(TARGET_SENTINEL)
    // position PID terms
    // PID terms for sentinel
    static constexpr float PID_17MM_P = 170000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 80.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr float PID_17MM_KICKER_P = 170000.0f;
    static constexpr float PID_17MM_KICKER_I = 0.0f;
    static constexpr float PID_17MM_KICKER_D = 80.0f;
    static constexpr float PID_17MM_KICKER_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_KICKER_MAX_OUT = 16000.0f;

    static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    static constexpr aruwlib::motor::MotorId SENTINEL_KICKER_MOTOR_ID = aruwlib::motor::MOTOR8;
    static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

#elif defined(TARGET_HERO)
    /// \todo tune all the things
    // PID terms for hero agitator 1
    static constexpr float PID_HERO1_P = 1500.0f;
    static constexpr float PID_HERO1_I = 500.0f;
    static constexpr float PID_HERO1_D = 7000.0f;
    static constexpr float PID_HERO1_MAX_ERR_SUM = 0.0f;

    // PID terms for hero agitator 2
    static constexpr float PID_HERO2_P = 1500.0f;
    static constexpr float PID_HERO2_I = 500.0f;
    static constexpr float PID_HERO2_D = 7000.0f;
    static constexpr float PID_HERO2_MAX_ERR_SUM = 0.0f;

    static constexpr aruwlib::motor::MotorId HERO1_AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    static constexpr aruwlib::can::CanBus HERO1_AGITATOR_MOTOR_CAN_BUS =
        aruwlib::can::CanBus::CAN_BUS1;

    static constexpr aruwlib::motor::MotorId HERO2_AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR6;
    static constexpr aruwlib::can::CanBus HERO2_AGITATOR_MOTOR_CAN_BUS =
        aruwlib::can::CanBus::CAN_BUS1;
#endif

    /**
     * Agitator gear ratios of different motors, for determining shaft rotation angle.
     */
    static constexpr float AGITATOR_GEAR_RATIO_M2006 = 36.0f;
    static constexpr float AGITATOR_GEAR_RATIO_GM3508 = 19.0f;

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
        bool isAgitatorInverted);

    void initialize() override;

    void refresh() override;

    /**
     * Sets desired angle in radians of the agitator motor, relative to where the agitator
     * has been initialized.
     *
     * @param[in] newAngle The desired angle.
     */
    void setAgitatorDesiredAngle(float newAngle);

    /**
     * @return The calibrated agitator angle, in radians. If the agitator is uncalibrated, 0
     *      radians is returned.
     */
    float getAgitatorAngle() const;

    /**
     * @return The angle set in `setAgitatorDesiredAngle`.
     */
    float getAgitatorDesiredAngle() const;

    /**
     * Attempts to calibrate the agitator at the current position, such that
     * `getAgitatorAngle` will return 0 radians at this position.
     *
     * @return `true` if the agitator has been successfully calibrated, `false`
     *      otherwise.
     */
    bool agitatorCalibrateHere();

    /**
     * A timer system may be used for determining if an agitator is jammed. This function
     * starts the agitator unjam timer. Call when starting to rotate to a position. Use
     * `isAgitatorJammed` to check the timer. When the agitator has reached a the desired
     * position, stop the unjam timer by calling `armAgitatorUnjamTimer`.
     *
     * @note In addition to the `predictedRotateTime`, an `JAMMED_TOLERANCE_PERIOD` is added
     *      to the timer's timeout.
     * @param[in] predictedRotateTime The time that you expect that agitator to rotate.
     */
    void armAgitatorUnjamTimer(uint32_t predictedRotateTime);

    /**
     * Stops the agitator unjam timer.
     */
    void disarmAgitatorUnjamTimer();

    /**
     * @return `true` if the agitator unjam timer has expired, signaling that the agitator
     *      has jammed, `false` otherwise.
     */
    bool isAgitatorJammed() const;

    /**
     * @return `true` if the agitator has been calibrated (`agitatorCalibrateHere` has been
     *      called and the agitator motor is online.
     */
    bool isAgitatorCalibrated() const;

    /**
     * @return The velocity of the agitator in units of degrees per second.
     */
    float getAgitatorVelocity() const;

private:
    /**
     * We add on this amount of "tolerance" to the predicted rotate time since some times it
     * takes longer than predicted and we only want to unjam when we are actually jammed.
     * Measured in ms.
     */
    static constexpr uint32_t JAMMED_TOLERANCE_PERIOD = 150;

    /**
     * PID controller for running postiion PID on unwrapped agitator angle (in radians).
     */
    aruwsrc::algorithms::TurretPid agitatorPositionPid;

    aruwlib::motor::DjiMotor agitatorMotor;

    /**
     * The user desired angle, measured in radians.
     * The agitator uses unwrapped angle.
     */
    float desiredAgitatorAngle;

    /**
     * You can calibrate the agitator, which will set the current agitator angle to zero radians.
     */
    float agitatorCalibratedZeroAngle;

    /**
     * Whether or not the agitator has been calibrated yet. You should calibrate the agitator
     * before using it.
     */
    bool agitatorIsCalibrated;

    /**
     * A timeout that is used to determine whether or not the agitator is jammed. If the
     * agitator has not reached the desired position in a certain time, the agitator is
     * considered jammed. units: milliseconds
     */
    aruwlib::arch::MilliTimeout agitatorJammedTimeout;

    /**
     * The current agitator timeout time, in milliseconds.
     */
    uint32_t agitatorJammedTimeoutPeriod;

    /**
     * Motor gear ratio, so we use shaft angle rather than encoder angle.
     */
    float gearRatio;

    void agitatorRunPositionPid();

    float getUncalibratedAgitatorAngle() const;
};  // class AgitatorSubsystem

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_SUBSYSTEM_HPP_
