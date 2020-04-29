#include <modm/architecture/interface/assert.h>
#include <modm/container/linked_list.hpp>
#include "aruwlib/motor/dji_motor_tx_handler.hpp"
#include "aruwlib/errors/create_errors.hpp"
#include "can_rx_handler.hpp"

namespace aruwlib
{

namespace can
{
    CanRxListner* CanRxHandler::messageHandlerStoreCan1[MAX_RECEIVE_UNIQUE_HEADER_CAN1] = {0};
    CanRxListner* CanRxHandler::messageHandlerStoreCan2[MAX_RECEIVE_UNIQUE_HEADER_CAN2] = {0};

    void CanRxHandler::attachReceiveHandler(CanRxListner *const CanRxHndl)
    {
        if (CanRxHndl->canBus == can::CanBus::CAN_BUS1)
        {
            attachReceiveHandler(CanRxHndl, messageHandlerStoreCan1,
                MAX_RECEIVE_UNIQUE_HEADER_CAN1);
        }
        else
        {
            attachReceiveHandler(CanRxHndl, messageHandlerStoreCan2,
                MAX_RECEIVE_UNIQUE_HEADER_CAN2);
        }
    }

    void CanRxHandler::attachReceiveHandler(
        CanRxListner *const CanRxHndl,
        CanRxListner** messageHandlerStore,
        const int16_t messageHandlerStoreSize
    ) {
        int8_t id = DJI_MOTOR_NORMALIZED_ID(CanRxHndl->canIdentifier);
        bool receiveInterfaceOverloaded = messageHandlerStore[id] != nullptr;
        bool receiveAttachSuccess = !receiveInterfaceOverloaded ||
            (id >= 0 && id < messageHandlerStoreSize);
        modm_assert(receiveAttachSuccess, "can1", "receive init", "overloading", 1);

        messageHandlerStore[id] = CanRxHndl;
    }

    void CanRxHandler::pollCanData()
    {
        modm::can::Message rxMessage;
        // handle incoming CAN 1 messages
        if (Can::getMessage(CanBus::CAN_BUS1, &rxMessage))
        {
            processReceivedCanData(rxMessage, messageHandlerStoreCan1);
        }
        // handle incoming CAN 2 messages
        if (Can::getMessage(CanBus::CAN_BUS2, &rxMessage))
        {
            processReceivedCanData(rxMessage, messageHandlerStoreCan2);
        }
    }

    void CanRxHandler::processReceivedCanData(
        const modm::can::Message& rxMessage,
        CanRxListner *const* messageHandlerStore
    ) {
        int32_t handlerStoreId = DJI_MOTOR_NORMALIZED_ID(rxMessage.getIdentifier());
        if ((handlerStoreId >= 0 && handlerStoreId < MAX_RECEIVE_UNIQUE_HEADER_CAN1)
            && messageHandlerStore[handlerStoreId] != nullptr)
        {
            messageHandlerStore[handlerStoreId]->processMessage(rxMessage);
        }
        else
        {
            RAISE_ERROR("Invalid can id received - not between 0x200 and 0x208",
                    aruwlib::errors::Location::CAN_RX,
                    aruwlib::errors::ErrorType::MOTOR_ID_OUT_OF_BOUNDS);
        }
    }

    void CanRxHandler::removeReceiveHandler(const CanRxListner& rxListner)
    {
        if (rxListner.canBus == CanBus::CAN_BUS1)
        {
            remoteReceiveHandler(rxListner, messageHandlerStoreCan1);
        }
        else
        {
            remoteReceiveHandler(rxListner, messageHandlerStoreCan2);
        }
    }

    void CanRxHandler::remoteReceiveHandler(
        const CanRxListner& rxListner,
        CanRxListner** messageHandlerStore
    ) {
        uint32_t id = DJI_MOTOR_NORMALIZED_ID(rxListner.canIdentifier);
        if (messageHandlerStore[id] == nullptr)
        {
            RAISE_ERROR("trying to remove something from rx listner that doesn't exist",
                    aruwlib::errors::CAN_RX, aruwlib::errors::INVALID_REMOVE)
            // error, trying to remove something that doesn't exist!
            return;
        }
        messageHandlerStore[id] = nullptr;
    }
}  // namespace can

}  // namespace aruwlib
