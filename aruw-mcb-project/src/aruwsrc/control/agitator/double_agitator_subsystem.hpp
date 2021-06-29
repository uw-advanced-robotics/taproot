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

#ifndef DOUBLE_AGITATOR_SUBSYSTEM_HPP_
#define DOUBLE_AGITATOR_SUBSYSTEM_HPP_

#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/control/setpoint/algorithms/setpoint_continuous_jam_checker.hpp"
#include "aruwlib/control/setpoint/interfaces/setpoint_subsystem.hpp"
#include "aruwlib/drivers.hpp"
#include "aruwlib/motor/dji_motor.hpp"

#include "agitator_subsystem.hpp"
#include "util_macros.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * A class for driving a shaft driven by two motors as if there was
 * only one motor. Functionally the interface is meant to
 * mimic that of the agitator subsystem. (Assumes that both motors
 * are facing each other, i.e.: positive rotation of the axis for one
 * would correspond to negative rotation on the other, and that both
 * have the same gear ratio and same output torque for a given input
 * value. In a nutshell, they should be identical, and honestly if they
 * aren't, then what the heck is the ME team doing).
 */
class DoubleAgitatorSubsystem : public aruwlib::control::setpoint::SetpointSubsystem
{
public:
    /**
     * The angular difference between the current angle and desired angle within which
     * the jam timer will be reset (i.e.: the subsystem won't consider itself jammed)
     * in radians (? I think)
     */
    static constexpr float JAM_DISTANCE_TOLERANCE = 0.5f;
    /**
     * The jam timeout period in milliseconds. Timeout starts and runs while subsystem
     * is outside JAM_DISTANCE_TOLERANCE. Timeout resets when subsystem returns to within tolerance.
     */
    static constexpr uint32_t JAM_TEMPORAL_TOLERANCE = 150;

    DoubleAgitatorSubsystem(
        aruwlib::Drivers *drivers,
        float kp,
        float ki,
        float kd,
        float maxIAccum,
        float maxOutput,
        float agitatorGearRatio,
        aruwlib::motor::MotorId agitator1MotorId,
        aruwlib::can::CanBus agitator1CanBusId,
        aruwlib::motor::MotorId agitator2MotorId,
        aruwlib::can::CanBus agitator2CanBusId,
        bool isAgitatorInverted,
        float jamDistanceTolerance = JAM_DISTANCE_TOLERANCE,
        uint32_t jamTemporalTolerance = JAM_TEMPORAL_TOLERANCE,
        bool jamLogicEnabled = true);

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
    mockable inline bool isJammed() override { return jamLogicEnabled && subsystemJamStatus; };

    /**
     * Clear the jam status of the subsystem, indicating that it has been unjammed.
     */
    mockable inline void clearJam() override { subsystemJamStatus = false; }

    /**
     * @return `true` if the agitator has been calibrated (`calibrateHere` has been
     *      called and the agitator motor is online).
     */
    mockable inline bool isCalibrated() override { return agitatorIsCalibrated; }

    /**
     * @return `true` if the subsystem is online (i.e.: is connected)
     *
     * @note currently return true only if both motors are online.
     */
    mockable inline bool isOnline() override
    {
        return agitatorMotor1.isMotorOnline() && agitatorMotor2.isMotorOnline();
    }

    /**
     * @return The velocity of the agitator in units of degrees per second.
     */
    mockable inline float getVelocity() override
    {
        return (getVelocity(agitatorMotor1) + getVelocity(agitatorMotor2)) / 2.0f;
    }

    void runHardwareTests() override;

    void onHardwareTestStart() override;

    mockable const char *getName() override { return "Double Agitator"; }

private:
    /**
     * The jam detection object. @note should be checked once per refresh
     */
    aruwlib::control::setpoint::SetpointContinuousJamChecker jamChecker;

    /**
     * PID controller for running postiion PID on unwrapped agitator angle (in radians).
     */
    aruwsrc::algorithms::SmoothPid agitatorPositionPid1;
    aruwsrc::algorithms::SmoothPid agitatorPositionPid2;

    /**
     * First of two motors driving the shaft on the double agitator subsystem
     */
    aruwlib::motor::DjiMotor agitatorMotor1;

    /**
     * Second of two motors driving the shaft on the double agitator subsystem.
     */
    aruwlib::motor::DjiMotor agitatorMotor2;

    /**
     * Stores whether or not the agitator has been calibrated.
     * Subsystem will auto-calibrate before outputting to motors.
     */
    bool agitatorIsCalibrated;

    /**
     * The user desired angle, measured in radians.
     * The agitator uses unwrapped angle.
     */
    float desiredAgitatorAngle = 0;

    /**
     * You can calibrate the agitator, which will set the current agitator angle to zero radians.
     */
    float agitator1CalibratedZeroAngle = 0;
    float agitator2CalibratedZeroAngle = 0;

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
     * Update the subsystem's PID and send appropriate outputs to motors
     */
    void agitatorRunPositionPid();

    inline float getVelocity(const aruwlib::motor::DjiMotor &motor) const
    {
        return 6.0f * static_cast<float>(motor.getShaftRPM()) / gearRatio;
    }

    /**
     * Get the raw shaft angle from one of the motors
     */
    inline float getUncalibratedAgitatorAngle(const aruwlib::motor::DjiMotor &motor) const
    {
        return (2.0f * aruwlib::algorithms::PI /
                static_cast<float>(aruwlib::motor::DjiMotor::ENC_RESOLUTION)) *
               motor.getEncoderUnwrapped() / gearRatio;
    }

    inline float getCurrentValue(const aruwlib::motor::DjiMotor &motor, float zeroOffset) const
    {
        return getUncalibratedAgitatorAngle(motor) - zeroOffset;
    }
};  // class DoubleAgitatorSubsystem

}  // namespace agitator

}  // namespace aruwsrc

#endif  // DOUBLE_AGITATOR_SUBSYSTEM_HPP_
