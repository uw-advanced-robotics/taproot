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

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/can_rx_handler_mock.hpp"
#include "tap/mock/can_rx_listener_mock.hpp"

using namespace testing;
using namespace tap::mock;

class CanRxHandlerTest : public Test
{
protected:
    CanRxHandlerTest() : handler(&drivers) {}

    void constructListeners()
    {
        for (int i = tap::motor::MOTOR1; i <= tap::motor::MOTOR8; i++)
        {
            int normalizedId = tap::can::CanRxHandler::lookupTableIndexForCanId(i);

            listeners[normalizedId] =
                new NiceMock<CanRxListenerMock>(&drivers, i, tap::can::CanBus::CAN_BUS1);
        }
    }

    void destructListeners()
    {
        for (int i = tap::motor::MOTOR1; i <= tap::motor::MOTOR8; i++)
        {
            int normalizedId = tap::can::CanRxHandler::lookupTableIndexForCanId(i);

            delete listeners[normalizedId];
        }
    }

    tap::Drivers drivers;
    tap::can::CanRxHandler handler;
    CanRxListenerMock *listeners[tap::motor::MOTOR8 - tap::motor::MOTOR1 + 1];
};

TEST(CanRxHandler, ListenerAttachesSelf)
{
    tap::Drivers drivers;
    CanRxListenerMock listener(&drivers, 0, tap::can::CanBus::CAN_BUS1);

    EXPECT_CALL(drivers.canRxHandler, attachReceiveHandler(&listener));
    EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler(testing::Ref(listener)));

    listener.attachSelfToRxHandler();
}

TEST_F(CanRxHandlerTest, ListenerAttachesAndDetatchesInArray)
{
    constructListeners();

    for (int i = tap::motor::MOTOR1; i <= tap::motor::MOTOR8; i++)
    {
        int normalizedId = tap::can::CanRxHandler::lookupTableIndexForCanId(i);

        handler.attachReceiveHandler(listeners[normalizedId]);

        EXPECT_EQ(
            listeners[normalizedId],
            handler.getHandlerStore(tap::can::CanBus::CAN_BUS1)[normalizedId]);

        handler.removeReceiveHandler(*listeners[normalizedId]);

        EXPECT_EQ(nullptr, handler.getHandlerStore(tap::can::CanBus::CAN_BUS1)[normalizedId]);
    }

    destructListeners();
}

// TEST_F(CanRxHandlerTest, MessageIsProcessedByCorrectListener)
// {
//     constructListeners();

//     for (int i = tap::motor::MOTOR1; i <= tap::motor::MOTOR8; i++)
//     {
//         int normalizedId = tap::can::CanRxHandler::lookupTableIndexForCanId(i);

//         EXPECT_CALL(*listeners[normalizedId], processMessage);
//     }

//     for (int i = tap::motor::MOTOR1; i <= tap::motor::MOTOR8; i++)
//     {
//         int normalizedId = tap::can::CanRxHandler::lookupTableIndexForCanId(i);

//         handler.attachReceiveHandler(listeners[normalizedId]);

//         const modm::can::Message rxMessage(i);
//         handler.processReceivedCanData(
//             rxMessage,
//             handler.getHandlerStore(tap::can::CanBus::CAN_BUS1));
//     }

//     destructListeners();
// }

TEST_F(CanRxHandlerTest, ErrorIsThrownWithOOBMessageID)
{
    const modm::can::Message rxMessage(9);

    EXPECT_CALL(drivers.errorController, addToErrorList);

    handler.processReceivedCanData(rxMessage, handler.getHandlerStore(tap::can::CanBus::CAN_BUS1));
}

TEST_F(CanRxHandlerTest, removeReceiveHandler__error_logged_with_oob_can_rx_listener_id)
{
    CanRxListenerMock canRxListenerHi(&drivers, 0xffff, tap::can::CanBus::CAN_BUS1);
    CanRxListenerMock canRxListenerLo(&drivers, 0x0, tap::can::CanBus::CAN_BUS1);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    handler.removeReceiveHandler(canRxListenerHi);
    handler.removeReceiveHandler(canRxListenerLo);
}
