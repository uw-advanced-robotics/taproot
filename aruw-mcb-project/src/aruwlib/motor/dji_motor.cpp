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

#include "dji_motor.hpp"

#include "aruwlib/Drivers.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"

namespace aruwlib
{
namespace motor
{
DjiMotor::~DjiMotor() { drivers->djiMotorTxHandler.removeFromMotorManager(*this); }

DjiMotor::DjiMotor(
    Drivers* drivers,
    MotorId desMotorIdentifier,
    aruwlib::can::CanBus motorCanBus,
    bool isInverted,
    const std::string& name)
    : CanRxListener(drivers, static_cast<uint32_t>(desMotorIdentifier), motorCanBus),
      encStore(),
      drivers(drivers),
      motorIdentifier(desMotorIdentifier),
      motorCanBus(motorCanBus),
      desiredOutput(0),
      shaftRPM(0),
      temperature(0),
      torque(0),
      motorInverted(isInverted),
      motorName(name)
{
    motorDisconnectTimeout.stop();
}

void DjiMotor::initialize()
{
    drivers->djiMotorTxHandler.addMotorToManager(this);
    attachSelfToRxHandler();
}

void DjiMotor::parseCanRxData(const modm::can::Message& message)
{
    if (message.getIdentifier() != DjiMotor::getMotorIdentifier())
    {
        return;
    }
    uint16_t encoderActual =
        static_cast<uint16_t>(message.data[0] << 8 | message.data[1]);        // encoder value
    shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
    shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
    torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
    torque = motorInverted ? -torque : torque;
    temperature = static_cast<int8_t>(message.data[6]);  // temperature

    // restart disconnect timer, since you just received a message from the motor
    motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

    // invert motor if necessary
    encoderActual = motorInverted ? ENC_RESOLUTION - 1 - encoderActual : encoderActual;
    encStore.updateValue(encoderActual);
}

void DjiMotor::setDesiredOutput(int32_t desiredOutput)
{
    int16_t desOutputNotInverted = static_cast<int16_t>(
        aruwlib::algorithms::limitVal<int32_t>(desiredOutput, SHRT_MIN, SHRT_MAX));
    this->desiredOutput = motorInverted ? -desOutputNotInverted : desOutputNotInverted;
}

bool DjiMotor::isMotorOnline() const
{
    /*
     * motor online if the disconnect timout has not expired (if it received message but
     * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout)
     * is stopped
     */
    return !motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped();
}

void DjiMotor::serializeCanSendData(modm::can::Message* txMessage) const
{
    int id = DJI_MOTOR_NORMALIZED_ID(this->getMotorIdentifier());  // number between 0 and 7
    // this method assumes you have choosen the correct message
    // to send the data in. Is blind to message type and is a private method
    // that I use accordingly.
    id %= 4;
    txMessage->data[2 * id] = this->getOutputDesired() >> 8;
    txMessage->data[2 * id + 1] = this->getOutputDesired() & 0xFF;
}

// getter functions
int16_t DjiMotor::getOutputDesired() const { return desiredOutput; }

uint32_t DjiMotor::getMotorIdentifier() const { return motorIdentifier; }

int8_t DjiMotor::getTemperature() const { return temperature; }

int16_t DjiMotor::getTorque() const { return torque; }

int16_t DjiMotor::getShaftRPM() const { return shaftRPM; }

bool DjiMotor::isMotorInverted() const { return motorInverted; }

aruwlib::can::CanBus DjiMotor::getCanBus() const { return motorCanBus; }

const std::string& DjiMotor::getName() const { return motorName; }

int64_t DjiMotor::EncoderStore::getEncoderUnwrapped() const
{
    return static_cast<int64_t>(encoderWrapped) +
           static_cast<int64_t>(ENC_RESOLUTION) * encoderRevolutions;
}

uint16_t DjiMotor::EncoderStore::getEncoderWrapped() const { return encoderWrapped; }

void DjiMotor::EncoderStore::updateValue(uint16_t newEncWrapped)
{
    int16_t enc_dif = newEncWrapped - encoderWrapped;
    if (enc_dif < -ENC_RESOLUTION / 2)
    {
        encoderRevolutions++;
    }
    else if (enc_dif > ENC_RESOLUTION / 2)
    {
        encoderRevolutions--;
    }
    encoderWrapped = newEncWrapped;
}
}  // namespace motor

}  // namespace aruwlib