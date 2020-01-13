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

    DjiMotor::DjiMotor(
        MotorId desMotorIdentifier,
        aruwlib::can::CanBus motorCanBus,
        bool isInverted
        ) : CanRxListner(static_cast<uint32_t>(desMotorIdentifier), motorCanBus),
        encStore(isInverted),
        motorIdentifier(desMotorIdentifier),
        motorCanBus(motorCanBus),
        desiredOutput(0),
        shaftRPM(0),
        temperature(0),
        torque(0),
        motorInverted(isInverted)
    {
        motorDisconnectTimeout.stop();
        DjiMotorTxHandler::addMotorToManager(this);
    }

    void DjiMotor::parseCanRxData(const modm::can::Message& message)
    {
        if (message.getIdentifier() != DjiMotor::getMotorIdentifier())
        {
            return;
        }
        uint16_t encoderActual = static_cast<uint16_t>
            (message.data[0] << 8 | message.data[1]);  // encoder value
        shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
        shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
        torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
        torque = motorInverted ? -torque : torque;
        temperature = static_cast<int8_t>(message.data[6]);  // temperature
        if (motorDisconnectTimeout.isStopped())  // the first time you receive a message, the
                                                 // motor disconnect timeout will be stopped
        {
            encStore.setInitialEncoderValue(encoderActual);
        }

        // restart disconnect timer, since you just received a message from the motor
        motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

        encStore.updateValue(encoderActual);

        this->encw = encStore.getEncoderWrapped();
    }

    void DjiMotor::setDesiredOutput(int32_t desiredOutput)
    {
        int16_t desOutputNotInverted = static_cast<int16_t>(aruwlib::algorithms::limitVal<int32_t>
            (desiredOutput, SHRT_MIN, SHRT_MAX));
        this->desiredOutput = motorInverted ? -desOutputNotInverted : desOutputNotInverted;
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
        txMessage->data[2 * id] = this->getOutputDesired() >> 8;
        txMessage->data[2 * id + 1] = this->getOutputDesired() & 0xFF;
    }

    // getter functions
    int16_t DjiMotor::getOutputDesired() const
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
        return motorInverted;
    }

    aruwlib::can::CanBus DjiMotor::getCanBus() const
    {
        return motorCanBus;
    }

    int32_t DjiMotor::EncoderStore::getEncoderUnwrapped() const
    {
        int32_t unwrappedNotInverted = encoderWrapped + ENC_RESOLUTION * encoderRevolutions;
        // to make the motor encoder value inverted, subtract the difference between the
        // encoder value you have that is not inverted by the first value you received from
        // the motor, and add this to the initial value again. doing this calculation this
        // way insures that when the motor is first connected, its starting encoder value
        // will be the same as a motor that is not inverted
        return encStoreInverted ?
            (2 * initialEncValue - unwrappedNotInverted) : unwrappedNotInverted;
    }

    int16_t DjiMotor::EncoderStore::getEncoderWrapped() const
    {
        if (!encStoreInverted) {
            return encoderWrapped;
        } else {
            int16_t val = 2 * initialEncValue - encoderWrapped;
            return val < 0 ? val + ENC_RESOLUTION + 1 : val;
        }
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

    void DjiMotor::EncoderStore::setInitialEncoderValue(uint16_t initEncValue)
    {
        initialEncValue = initEncValue;
    }
}  // namespace motor

}  // namespace aruwlib
