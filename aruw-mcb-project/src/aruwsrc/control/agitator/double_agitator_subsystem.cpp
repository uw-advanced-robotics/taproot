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

#include "double_agitator_subsystem.hpp"

using aruwlib::algorithms::PI;
using aruwlib::motor::DjiMotor;

namespace aruwsrc
{
namespace agitator
{
DoubleAgitatorSubsystem::DoubleAgitatorSubsystem(
    aruwlib::Drivers* drivers,
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
    float jamDistanceTolerance,
    uint32_t jamTemporalTolerance,
    bool jamLogicEnabled)
    : Subsystem(drivers),
      jamChecker(this, jamDistanceTolerance, jamTemporalTolerance),
      agitatorPositionPid1(kp, ki, kd, maxIAccum, maxOutput, 1.0f, 0.0f, 1.0f, 0.0f),
      agitatorPositionPid2(kp, ki, kd, maxIAccum, maxOutput, 1.0f, 0.0f, 1.0f, 0.0f),
      agitatorMotor1(
          drivers,
          agitator1MotorId,
          agitator1CanBusId,
          isAgitatorInverted,
          "agitator motor"),
      agitatorMotor2(
          drivers,
          agitator2MotorId,
          agitator2CanBusId,
          !isAgitatorInverted,
          "agitator motor"),
      gearRatio(agitatorGearRatio),
      jamLogicEnabled(jamLogicEnabled)
{
}

void DoubleAgitatorSubsystem::initialize()
{
    agitatorMotor1.initialize();
    agitatorMotor2.initialize();
}

void DoubleAgitatorSubsystem::refresh()
{
    if (!agitatorIsCalibrated)
    {
        calibrateHere();
    }

    agitatorRunPositionPid();
    if (jamChecker.check())
    {
        subsystemJamStatus = true;
    }
}

void DoubleAgitatorSubsystem::agitatorRunPositionPid()
{
    if (!agitatorIsCalibrated)
    {
        agitatorPositionPid1.reset();
        agitatorPositionPid2.reset();
    }
    else if (!isOnline())
    {
        agitatorPositionPid1.reset();
        agitatorPositionPid2.reset();
        agitatorIsCalibrated = false;
    }
    else
    {
        // dt doesn't need to be exact since we don't use an integral term and we calculate
        // the velocity ourselves, so it currently isn't used.
        float motor1Angle = getCurrentValue(agitatorMotor1, agitator1CalibratedZeroAngle);
        float motor2Angle = getCurrentValue(agitatorMotor2, agitator2CalibratedZeroAngle);
        float angleDiff = motor1Angle - motor2Angle;
        agitatorPositionPid1.runController(
            desiredAgitatorAngle - motor1Angle - angleDiff,
            getVelocity(agitatorMotor1),
            2.0f);
        agitatorPositionPid2.runController(
            desiredAgitatorAngle - motor2Angle + angleDiff,
            getVelocity(agitatorMotor2),
            2.0f);
        agitatorMotor1.setDesiredOutput(agitatorPositionPid1.getOutput());
        agitatorMotor2.setDesiredOutput(agitatorPositionPid2.getOutput());
    }
}

bool DoubleAgitatorSubsystem::calibrateHere()
{
    if (!isOnline())
    {
        return false;
    }
    agitator1CalibratedZeroAngle = getUncalibratedAgitatorAngle(agitatorMotor1);
    agitator2CalibratedZeroAngle = getUncalibratedAgitatorAngle(agitatorMotor2);
    agitatorIsCalibrated = true;
    desiredAgitatorAngle = 0.0f;
    return true;
}

float DoubleAgitatorSubsystem::getCurrentValue() const
{
    if (!agitatorIsCalibrated)
    {
        return 0.0f;
    }
    return (getCurrentValue(agitatorMotor1, agitator1CalibratedZeroAngle) +
            getCurrentValue(agitatorMotor2, agitator2CalibratedZeroAngle)) /
           2.0f;
}

void DoubleAgitatorSubsystem::runHardwareTests()
{
    if (aruwlib::algorithms::compareFloatClose(
            this->getSetpoint(),
            this->getCurrentValue(),
            PI / 16))
    {
        this->setHardwareTestsComplete();
    }
}

void DoubleAgitatorSubsystem::onHardwareTestStart()
{
    this->setSetpoint(this->getCurrentValue() + PI / 2);
}

}  // namespace agitator

}  // namespace aruwsrc
