/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "double_dji_motor.hpp"

namespace tap::motor
{
DoubleDjiMotor::DoubleDjiMotor(
    Drivers* drivers,
    MotorId desMotorIdentifierOne,
    MotorId desMotorIdentifierTwo,
    tap::can::CanBus motorCanBusOne,
    tap::can::CanBus motorCanBusTwo,
    bool isInvertedOne,
    bool isInvertedTwo,
    const char* nameOne,
    const char* nameTwo,
    uint16_t encWrapped,
    int64_t encRevolutions)
    : motorOne(
          drivers,
          desMotorIdentifierOne,
          motorCanBusOne,
          isInvertedOne,
          nameOne,
          encWrapped,
          encRevolutions),
      motorTwo(
          drivers,
          desMotorIdentifierTwo,
          motorCanBusTwo,
          isInvertedTwo,
          nameTwo,
          encWrapped,
          encRevolutions)
{
}

void DoubleDjiMotor::initialize()
{
    motorOne.initialize();
    motorTwo.initialize();
}

int64_t DoubleDjiMotor::getEncoderUnwrapped() const { 
    return callIfOnline(&DjiMotor::getEncoderUnwrapped);
}

uint16_t DoubleDjiMotor::getEncoderWrapped() const {
    return callIfOnline(&DjiMotor::getEncoderWrapped);
}

void DoubleDjiMotor::resetEncoderValue()
{
    motorOne.resetEncoderValue();
    motorTwo.resetEncoderValue();
}

float DoubleDjiMotor::getPositionUnwrapped() const { 
    return callIfOnline(&DjiMotor::getPositionUnwrapped);
}

float DoubleDjiMotor::getPositionWrapped() const {
    return callIfOnline(&DjiMotor::getPositionWrapped);
}

void DoubleDjiMotor::setDesiredOutput(int32_t desiredOutput)
{
    motorOne.setDesiredOutput(desiredOutput);
    motorTwo.setDesiredOutput(desiredOutput);
}

bool DoubleDjiMotor::isMotorOnline() const
{
    return motorOne.isMotorOnline() && motorTwo.isMotorOnline();
}

int16_t DoubleDjiMotor::getOutputDesired() const
{
    int16_t m1Out =
        motorOne.isMotorInverted() ? -motorOne.getOutputDesired() : motorOne.getOutputDesired();
    int16_t m2Out =
        motorTwo.isMotorInverted() ? -motorTwo.getOutputDesired() : motorTwo.getOutputDesired();

    return (static_cast<int32_t>(m1Out) + static_cast<int32_t>(m2Out)) / 2;
}

int8_t DoubleDjiMotor::getTemperature() const
{
    return std::max(motorOne.getTemperature(), motorTwo.getTemperature());
}
int16_t DoubleDjiMotor::getTorque() const
{
    int32_t m1Torque = motorOne.getTorque();
    int32_t m2Torque = motorTwo.getTorque();
    int num_online = motorOne.isMotorOnline() + motorTwo.isMotorOnline();
    
    return num_online == 0 ? 0 : (m1Torque + m2Torque) / num_online;
}

int16_t DoubleDjiMotor::getShaftRPM() const
{
    int m1RPM = motorOne.getShaftRPM();
    int m2RPM = motorTwo.getShaftRPM();
    int num_online = motorOne.isMotorOnline() + motorTwo.isMotorOnline();
    
    return num_online == 0 ? 0 : (m1RPM + m2RPM) / num_online;
}
}  // namespace tap::motor
