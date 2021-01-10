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

#include "agitator_subsystem.hpp"

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/errors/create_errors.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <modm/math/filter/pid.hpp>

#include "agitator_rotate_command.hpp"

using namespace aruwlib::motor;

namespace aruwsrc
{
namespace agitator
{
AgitatorSubsystem::AgitatorSubsystem(
    aruwlib::Drivers* drivers,
    float kp,
    float ki,
    float kd,
    float maxIAccum,
    float maxOutput,
    float agitatorGearRatio,
    aruwlib::motor::MotorId agitatorMotorId,
    aruwlib::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted)
    : aruwlib::control::Subsystem(drivers),
      agitatorPositionPid(kp, ki, kd, maxIAccum, maxOutput, 1.0f, 0.0f, 1.0f, 0.0f),
      agitatorMotor(
          drivers,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted,
          "agitator motor"),
      desiredAgitatorAngle(0.0f),
      agitatorCalibratedZeroAngle(0.0f),
      agitatorIsCalibrated(false),
      agitatorJammedTimeout(0),
      agitatorJammedTimeoutPeriod(0),
      gearRatio(agitatorGearRatio)
{
    agitatorJammedTimeout.stop();
}

void AgitatorSubsystem::initialize() { agitatorMotor.initialize(); }

void AgitatorSubsystem::armAgitatorUnjamTimer(uint32_t predictedRotateTime)
{
    if (predictedRotateTime == 0)
    {
        RAISE_ERROR(
            drivers,
            "The predicted rotate time is 0, this is physically impossible",
            aruwlib::errors::SUBSYSTEM,
            aruwlib::errors::ZERO_DESIRED_AGITATOR_ROTATE_TIME);
    }
    agitatorJammedTimeoutPeriod = predictedRotateTime + JAMMED_TOLERANCE_PERIOD;
    agitatorJammedTimeout.restart(agitatorJammedTimeoutPeriod);
}

void AgitatorSubsystem::disarmAgitatorUnjamTimer() { agitatorJammedTimeout.stop(); }

bool AgitatorSubsystem::isAgitatorJammed() const { return agitatorJammedTimeout.isExpired(); }

void AgitatorSubsystem::refresh()
{
    if (agitatorIsCalibrated)
    {
        agitatorRunPositionPid();
    }
    else
    {
        agitatorCalibrateHere();
    }
}

void AgitatorSubsystem::agitatorRunPositionPid()
{
    if (!agitatorIsCalibrated)
    {
        agitatorPositionPid.reset();
    }
    else if (!agitatorMotor.isMotorOnline())
    {
        agitatorPositionPid.reset();
        agitatorIsCalibrated = false;
    }
    else
    {
        agitatorPositionPid.runController(
            desiredAgitatorAngle - getAgitatorAngle(),
            getAgitatorVelocity());
        agitatorMotor.setDesiredOutput(agitatorPositionPid.getOutput());
    }
}

bool AgitatorSubsystem::agitatorCalibrateHere()
{
    if (!agitatorMotor.isMotorOnline())
    {
        return false;
    }
    agitatorCalibratedZeroAngle = getUncalibratedAgitatorAngle();
    agitatorIsCalibrated = true;
    return true;
}

float AgitatorSubsystem::getAgitatorAngle() const
{
    if (!agitatorIsCalibrated)
    {
        return 0.0f;
    }
    return getUncalibratedAgitatorAngle() - agitatorCalibratedZeroAngle;
}

float AgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    // position is equal to the following equation:
    // position = 2 * PI / encoder resolution * unwrapped encoder value / gear ratio
    return (2.0f * aruwlib::algorithms::PI / static_cast<float>(DjiMotor::ENC_RESOLUTION)) *
           agitatorMotor.encStore.getEncoderUnwrapped() / gearRatio;
}

void AgitatorSubsystem::setAgitatorDesiredAngle(float newAngle) { desiredAgitatorAngle = newAngle; }

float AgitatorSubsystem::getAgitatorDesiredAngle() const { return desiredAgitatorAngle; }

float AgitatorSubsystem::getAgitatorVelocity() const
{
    return 6.0f * static_cast<float>(agitatorMotor.getShaftRPM()) / gearRatio;
}

bool AgitatorSubsystem::isAgitatorCalibrated() const { return agitatorIsCalibrated; }

void AgitatorSubsystem::runHardwareTests()
{
    // TODO
}

}  // namespace agitator

}  // namespace aruwsrc
