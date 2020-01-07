#include "dji_motor.hpp"
#include "dji_motor_tx_handler.hpp"

namespace aruwlib
{

namespace motor
{
    DjiMotor::~DjiMotor()
    {
        DjiMotorTxHandler::removeFromMotorManager(*this);
    }

    DjiMotor::DjiMotor(MotorId desMotorIdentifier, aruwlib::can::CanBus motorCanBus)
        : CanRxListner(static_cast<uint32_t>(desMotorIdentifier), motorCanBus),
        motorIdentifier(desMotorIdentifier),
        motorCanBus(motorCanBus),
        desiredOutput(0),
        currentActual(0),
        shaftRPM(0),
        temperature(0),
        torque(0),
        motorOnline(false)
    {
        motorDisconnectTimeout.stop();
        DjiMotorTxHandler::addMotorToManager(this);
    }

    void DjiMotor::parseCanRxData(const modm::can::Message& message)
    {
        // restart disconnect timer, since you just received a message from the motor
        motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

        if (message.getIdentifier() != DjiMotor::getMotorIdentifier())
        {
            return;
        }
        uint16_t encoderActual = static_cast<uint16_t>
            (message.data[0] << 8 | message.data[1]);  // encoder value
        shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
        torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
        temperature = static_cast<int8_t>(message.data[6]);  // temperature
        motorOnline = true;  // when first message is received, motor is now online

        encStore.updateValue(encoderActual);
    }

    void DjiMotor::setDesiredOutput(int32_t desiredOutput)
    {
        this->desiredOutput =
            static_cast<int16_t>(algorithms::limitVal<int32_t>(desiredOutput, SHRT_MIN, SHRT_MAX));
    }

    bool DjiMotor::isMotorOnline()
    {
        return !motorDisconnectTimeout.isExpired() || motorDisconnectTimeout.isStopped();
    }

    void DjiMotor::serializeCanSendData(modm::can::Message* txMessage) const
    {
        int id = DJI_MOTOR_NORMALIZED_ID(this->getMotorIdentifier());  // number between 0 and 7
        // this method assumes you have choosen the correct message
        // to send the data in. Is blind to message type and is a private method
        // that I use accordingly.
        id %= 4;
        txMessage->data[2 * id] = this->getVoltageDesired() >> 8;
        txMessage->data[2 * id + 1] = this->getVoltageDesired() & 0xFF;
    }

    // getter functions
    int16_t DjiMotor::getVoltageDesired() const
    {
        return desiredOutput;
    }

    uint32_t DjiMotor::getMotorIdentifier() const
    {
        return motorIdentifier;
    }

    int8_t DjiMotor::getTemperature() const
    {
        return temperature;
    }

    int16_t DjiMotor::getTorque() const
    {
        return torque;
    }

    int16_t DjiMotor::getShaftRPM() const
    {
        return shaftRPM;
    }

    int16_t DjiMotor::getCurrentActual() const
    {
        return currentActual;
    }

    aruwlib::can::CanBus DjiMotor::getCanBus() const
    {
        return motorCanBus;
    }

    int32_t DjiMotor::EncoderStore::getEncoderUnwrapped() const
    {
        return encoderWrapped + ENC_RESOLUTION * encoderRevolutions;
    }

    int16_t DjiMotor::EncoderStore::getEncoderWrapped() const
    {
        return encoderWrapped;
    }

    void DjiMotor::EncoderStore::updateValue(uint16_t newEncWrapped)
    {
        int16_t enc_dif = newEncWrapped - encoderWrapped;
        if(enc_dif < -ENC_RESOLUTION / 2)
        {
            encoderRevolutions++;
        }
        else if(enc_dif > ENC_RESOLUTION / 2)
        {
            encoderRevolutions--;
        }
        encoderWrapped = newEncWrapped;
    }
}  // namespace motor

}  // namespace aruwlib
