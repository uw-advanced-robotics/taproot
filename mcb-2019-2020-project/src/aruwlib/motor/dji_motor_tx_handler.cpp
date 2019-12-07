#include <modm/architecture/interface/assert.h>
#include <rm-dev-board-a/board.hpp>
#include "dji_motor_tx_handler.hpp"
#include "dji_motor.hpp"

#define CAN_DJI_MESSAGE_SEND_LENGTH 8
#define CAN_DJI_LOW_IDENTIFIER 0X200
#define CAN_DJI_HIGH_IDENTIFIER 0X1FF

namespace aruwlib
{

namespace motor
{
    #define DJI_MOTORS_PER_CAN 8

    DjiMotor* can1MotorStore[DJI_MOTORS_PER_CAN] = {0};
    DjiMotor* can2MotorStore[DJI_MOTORS_PER_CAN] = {0};

    void DjiMotorTxHandler::addMotorToManager(DjiMotor** canMotorStore, DjiMotor*const motor)
    {
        int16_t idIndex = DJI_MOTOR_NORMALIZED_ID(motor->getMotorIdentifier());
        bool motorOverloaded = canMotorStore[idIndex] != nullptr;
        bool motorOutOfBounds = (idIndex < 0) || (idIndex >= DJI_MOTORS_PER_CAN);
        // kill start
        modm_assert(!motorOverloaded && !motorOutOfBounds, "can", "motor init", "overloading", 1);
        canMotorStore[idIndex] = motor;
    }

    void DjiMotorTxHandler::addMotorToManager(DjiMotor* motor)
    {
        // add new motor to either the can1 or can2 motor store
        // because we checked to see if the motor is overloaded, we will
        // never have to worry about overfilling the CanxMotorStore array
        if (motor->getCanBus() == aruwlib::can::CanBus::CAN_BUS1)
        {
            addMotorToManager(can1MotorStore, motor);
        }
        else
        {
            addMotorToManager(can2MotorStore, motor);
        }
    }

    // cppcheck-suppress unusedFunction //TODO Remove lint suppression
    void DjiMotorTxHandler::processCanSendData()
    {
        // set up new can messages to be sent via CAN bus 1 and 2
        modm::can::Message can1MessageLow(
            CAN_DJI_LOW_IDENTIFIER,
            CAN_DJI_MESSAGE_SEND_LENGTH
        );
        can1MessageLow.setExtended(false);

        modm::can::Message can1MessageHigh(
            CAN_DJI_HIGH_IDENTIFIER,
            CAN_DJI_MESSAGE_SEND_LENGTH
        );
        can1MessageHigh.setExtended(false);

        modm::can::Message can2MessageLow(
            CAN_DJI_LOW_IDENTIFIER,
            CAN_DJI_MESSAGE_SEND_LENGTH
        );
        can2MessageLow.setExtended(false);

        modm::can::Message can2MessageHigh(
            CAN_DJI_HIGH_IDENTIFIER,
            CAN_DJI_MESSAGE_SEND_LENGTH
        );
        can2MessageHigh.setExtended(false);

        serializeMotorStoreSendData(can1MotorStore, &can1MessageLow, &can1MessageHigh);
        serializeMotorStoreSendData(can2MotorStore, &can2MessageLow, &can2MessageHigh);

        if (Can1::isReadyToSend())
        {
            Can1::sendMessage(can1MessageLow);
            Can1::sendMessage(can1MessageHigh);
        }
        if (Can2::isReadyToSend())
        {
            Can2::sendMessage(can2MessageLow);
            Can2::sendMessage(can2MessageHigh);
        }
    }

    void DjiMotorTxHandler::serializeMotorStoreSendData(
        DjiMotor** canMotorStore,
        modm::can::Message* messageLow,
        modm::can::Message* messageHigh
    ) {
        for (int i = 0; i < DJI_MOTORS_PER_CAN; i++)
        {
            const DjiMotor*const motor = canMotorStore[i];
            if (motor != nullptr)
            {
                if (motor->getMotorIdentifier() - 0x200 <= 4)
                {
                    motor->serializeCanSendData(messageLow);
                }
                else
                {
                    motor->serializeCanSendData(messageHigh);
                }
            }
        }
    }

    void DjiMotorTxHandler::removeFromMotorManager(const DjiMotor& motor)
    {
        if (motor.getCanBus() == aruwlib::can::CanBus::CAN_BUS1)
        {
            removeFromMotorManager(motor, can1MotorStore);
        }
        else
        {
            removeFromMotorManager(motor, can2MotorStore);
        }
    }

    void DjiMotorTxHandler::removeFromMotorManager(const DjiMotor& motor, DjiMotor** motorStore)
    {
        uint32_t id = DJI_MOTOR_NORMALIZED_ID(motor.getMotorIdentifier());
        if (motorStore[id] == nullptr)
        {
            // error, trying to remove something that doesn't exist!
            // NON-FATAL-ERROR-CHECK
            return;
        }
        motorStore[id] = nullptr;
    }

}  // namespace motor

}  // namespace aruwlib
