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

#include "can_rx_handler.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/assert.h"
#include "modm/architecture/interface/can.hpp"

#include "can_rx_listener.hpp"

namespace tap::can
{
CanRxHandler::CanRxHandler(Drivers* drivers)
    : drivers(drivers),
      messageHandlerStoreCan1(),
      messageHandlerStoreCan2()
{
}

void CanRxHandler::attachReceiveHandler(CanRxListener* const listener)
{
    if (listener->canBus == can::CanBus::CAN_BUS1)
    {
        attachReceiveHandler(listener, messageHandlerStoreCan1);
    }
    else
    {
        attachReceiveHandler(listener, messageHandlerStoreCan2);
    }
}

void CanRxHandler::attachReceiveHandler(
    CanRxListener* const canRxListener,
    CanRxListener** messageHandlerStore)
{
    uint16_t id = normalizeCanId(canRxListener->canIdentifier);

    bool receiveInterfaceOverloaded = messageHandlerStore[id] != nullptr;
    modm_assert(!receiveInterfaceOverloaded || (id < MAX_CAN_ID), "can1", "overloading", 1);

    messageHandlerStore[id] = canRxListener;
}

void CanRxHandler::pollCanData()
{
    modm::can::Message rxMessage;

    // handle incoming CAN 1 messages
    if (drivers->can.getMessage(CanBus::CAN_BUS1, &rxMessage))
    {
        processReceivedCanData(rxMessage, messageHandlerStoreCan1);
    }

    // handle incoming CAN 2 messages
    if (drivers->can.getMessage(CanBus::CAN_BUS2, &rxMessage))
    {
        processReceivedCanData(rxMessage, messageHandlerStoreCan2);
    }
}

void CanRxHandler::processReceivedCanData(
    const modm::can::Message& rxMessage,
    CanRxListener* const* messageHandlerStore)
{
    uint16_t id = normalizeCanId(rxMessage.getIdentifier());

    if (id >= MAX_CAN_ID)
    {
        RAISE_ERROR(
            drivers,
            "Invalid can id received",
            tap::errors::Location::CAN_RX,
            tap::errors::CanRxErrorType::MOTOR_ID_OUT_OF_BOUNDS);
    }

    if (messageHandlerStore[id] != nullptr)
    {
        messageHandlerStore[id]->processMessage(rxMessage);
    }
}

void CanRxHandler::removeReceiveHandler(const CanRxListener& canRxListener)
{
    if (canRxListener.canBus == CanBus::CAN_BUS1)
    {
        removeReceiveHandler(canRxListener, messageHandlerStoreCan1);
    }
    else
    {
        removeReceiveHandler(canRxListener, messageHandlerStoreCan2);
    }
}

void CanRxHandler::removeReceiveHandler(
    const CanRxListener& canRxListener,
    CanRxListener** messageHandlerStore)
{
    int id = normalizeCanId(canRxListener.canIdentifier);

    if (id >= MAX_CAN_ID)
    {
        RAISE_ERROR(
            drivers,
            "index out of bounds",
            tap::errors::CAN_RX,
            tap::errors::CanRxErrorType::INVALID_REMOVE);
        return;
    }

    messageHandlerStore[id] = nullptr;
}

}  // namespace can
