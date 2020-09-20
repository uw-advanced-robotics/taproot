/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "turret_subsystem.hpp"

#include <algorithm>
#include <cfloat>
#include <random>

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/clock.hpp>
#include <aruwlib/control/command_mapper.hpp>
#include <aruwlib/errors/create_errors.hpp>

using namespace aruwlib::motor;
using namespace aruwlib;

namespace aruwsrc
{
namespace turret
{
TurretSubsystem::TurretSubsystem(aruwlib::Drivers* drivers)
    : aruwlib::control::Subsystem(drivers),
      pitchMotor(drivers, PITCH_MOTOR_ID, CAN_BUS_MOTORS, true, "pitch motor"),
      yawMotor(drivers, YAW_MOTOR_ID, CAN_BUS_MOTORS, false, "yaw motor"),
      currPitchAngle(0.0f, 0.0f, 360.0f),
      currYawAngle(0.0f, 0.0f, 360.0f),
      yawTarget(TURRET_START_ANGLE, 0.0f, 360.0f),
      pitchTarget(TURRET_START_ANGLE, 0.0f, 360.0f)
{
}

void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
}

float TurretSubsystem::getYawAngleFromCenter() const
{
    aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
        currYawAngle.getValue() - TURRET_START_ANGLE,
        -180.0f,
        180.0f);
    return yawAngleFromCenter.getValue();
}

float TurretSubsystem::getPitchAngleFromCenter() const
{
    aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
        currPitchAngle.getValue() - TURRET_START_ANGLE,
        -180.0f,
        180.0f);
    return yawAngleFromCenter.getValue();
}

int32_t TurretSubsystem::getYawVelocity() const
{
    if (!yawMotor.isMotorOnline())
    {
        RAISE_ERROR(
            drivers,
            "trying to get velocity and yaw motor offline",
            aruwlib::errors::TURRET,
            aruwlib::errors::MOTOR_OFFLINE);
        // throw error
        return 0;
    }

    return getVelocity(yawMotor);
}

int32_t TurretSubsystem::getPitchVelocity() const
{
    if (!pitchMotor.isMotorOnline())
    {
        RAISE_ERROR(
            drivers,
            "trying to get velocity and pitch motor offline",
            aruwlib::errors::TURRET,
            aruwlib::errors::MOTOR_OFFLINE);
        return 0;
    }

    return getVelocity(pitchMotor);
}

// units: degrees per second
int32_t TurretSubsystem::getVelocity(const DjiMotor& motor) const
{
    return 360 / 60 * motor.getShaftRPM();
}

bool TurretSubsystem::isTurretOnline() const
{
    return pitchMotor.isMotorOnline() && yawMotor.isMotorOnline();
}

void TurretSubsystem::refresh() { updateCurrentTurretAngles(); }

void TurretSubsystem::updateCurrentTurretAngles()
{
    updateCurrentYawAngle();
    updateCurrentPitchAngle();
}

void TurretSubsystem::updateCurrentYawAngle()
{
    if (yawMotor.isMotorOnline())
    {
        currYawAngle.setValue(
            DjiMotor::encoderToDegrees(static_cast<uint16_t>(
                yawMotor.encStore.getEncoderWrapped() - YAW_START_ENCODER_POSITION)) +
            TURRET_START_ANGLE);
    }
    else
    {
        currYawAngle.setValue(TURRET_START_ANGLE);
    }
}

void TurretSubsystem::updateCurrentPitchAngle()
{
    if (pitchMotor.isMotorOnline())
    {
        currPitchAngle.setValue(
            DjiMotor::encoderToDegrees(static_cast<uint16_t>(
                pitchMotor.encStore.getEncoderWrapped() - PITCH_START_ENCODER_POSITION)) +
            TURRET_START_ANGLE);
    }
    else
    {
        currPitchAngle.setValue(TURRET_START_ANGLE);
    }
}

void TurretSubsystem::setPitchMotorOutput(float out)
{
    if (out > INT32_MAX || out < INT32_MIN)
    {
        RAISE_ERROR(
            drivers,
            "pitch motor output invalid",
            aruwlib::errors::TURRET,
            aruwlib::errors::INVALID_MOTOR_OUTPUT);
        return;
    }
    if (pitchMotor.isMotorOnline())
    {
        if ((getPitchAngleFromCenter() + TURRET_START_ANGLE > TURRET_PITCH_MAX_ANGLE && out > 0) ||
            (getPitchAngleFromCenter() + TURRET_START_ANGLE < TURRET_PITCH_MIN_ANGLE && out < 0))
        {
            pitchMotor.setDesiredOutput(0);
        }
        else
        {
            pitchMotor.setDesiredOutput(out);
        }
    }
}

void TurretSubsystem::setYawMotorOutput(float out)
{
    if (out > INT32_MAX || out < INT32_MIN)
    {
        RAISE_ERROR(
            drivers,
            "yaw motor output invalid",
            aruwlib::errors::TURRET,
            aruwlib::errors::INVALID_MOTOR_OUTPUT);
        return;
    }
    if (yawMotor.isMotorOnline())
    {
        if ((getYawAngleFromCenter() + TURRET_START_ANGLE > TURRET_YAW_MAX_ANGLE && out > 0) ||
            (getYawAngleFromCenter() + TURRET_START_ANGLE < TURRET_YAW_MIN_ANGLE && out < 0))
        {
            yawMotor.setDesiredOutput(0);
        }
        else
        {
            yawMotor.setDesiredOutput(out);
        }
    }
}

const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getYawAngle() const
{
    return currYawAngle;
}

const aruwlib::algorithms::ContiguousFloat& TurretSubsystem::getPitchAngle() const
{
    return currPitchAngle;
}

float TurretSubsystem::getYawTarget() const { return yawTarget.getValue(); }

float TurretSubsystem::getPitchTarget() const { return pitchTarget.getValue(); }

void TurretSubsystem::setYawTarget(float target)
{
    yawTarget.setValue(target);
    yawTarget.setValue(aruwlib::algorithms::ContiguousFloat::limitValue(
        yawTarget,
        TURRET_YAW_MIN_ANGLE,
        TURRET_YAW_MAX_ANGLE));
}

void TurretSubsystem::setPitchTarget(float target)
{
    pitchTarget.setValue(target);
    pitchTarget.setValue(aruwlib::algorithms::ContiguousFloat::limitValue(
        pitchTarget,
        TURRET_PITCH_MIN_ANGLE,
        TURRET_PITCH_MAX_ANGLE));
}

float TurretSubsystem::yawFeedForwardCalculation(float desiredChassisRotation)
{
    // calculate feed forward
    float chassisRotationProportional =
        FEED_FORWARD_KP * desiredChassisRotation *
        (fabsf(
             FEED_FORWARD_SIN_GAIN *
             sinf(getYawAngleFromCenter() * aruwlib::algorithms::PI / 180.0f)) +
         1.0f);

    if (drivers->remote.getUpdateCounter() != prevUpdateCounterChassisRotateDerivative)
    {
        chassisRotateDerivativeInterpolation.update(
            desiredChassisRotation - feedforwardPrevChassisRotationDesired);
    }
    prevUpdateCounterChassisRotateDerivative = drivers->remote.getUpdateCounter();
    float derivativeInterpolated = chassisRotateDerivativeInterpolation.getInterpolatedValue(
        aruwlib::arch::clock::getTimeMilliseconds());

    feedforwardChassisRotateDerivative = aruwlib::algorithms::lowPassFilter(
        feedforwardChassisRotateDerivative,
        derivativeInterpolated,
        FEED_FORWARD_DERIVATIVE_LOW_PASS);

    float chassisRotationFeedForward = aruwlib::algorithms::limitVal<float>(
        chassisRotationProportional + FEED_FORWARD_KD * feedforwardChassisRotateDerivative,
        -FEED_FORWARD_MAX_OUTPUT,
        FEED_FORWARD_MAX_OUTPUT);

    feedforwardPrevChassisRotationDesired = desiredChassisRotation;

    if ((chassisRotationFeedForward > 0.0f &&
         getYawAngle().getValue() > TurretSubsystem::TURRET_YAW_MAX_ANGLE) ||
        (chassisRotationFeedForward < 0.0f &&
         getYawAngle().getValue() < TurretSubsystem::TURRET_YAW_MIN_ANGLE))
    {
        chassisRotationFeedForward = 0.0f;
    }
    return chassisRotationFeedForward;
}
}  // namespace turret

}  // namespace aruwsrc
