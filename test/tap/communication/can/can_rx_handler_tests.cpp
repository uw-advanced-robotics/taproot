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

#include <memory>

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/can_rx_handler_mock.hpp"
#include "tap/mock/can_rx_listener_mock.hpp"

using namespace testing;
using namespace std;
using namespace tap::mock;

class CanRxHandlerTest : public Test
{
protected:
    CanRxHandlerTest() : handler(&drivers) {}

    void constructListeners()
    {
        for (uint32_t i = tap::motor::MOTOR1; i <= tap::motor::MOTOR8; i++)
        {
            auto listener = make_unique<CanRxListenerMock>(&drivers, i, tap::can::CanBus::CAN_BUS1);
            listeners.push_back(move(listener));
        }
    }

    tap::Drivers drivers;
    tap::can::CanRxHandler handler;
    vector<unique_ptr<CanRxListenerMock>> listeners;
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

    for (auto &listener : listeners)
    {
        handler.attachReceiveHandler(listener.get());

        int normalizedId =
            tap::can::CanRxHandler::lookupTableIndexForCanId(listener->canIdentifier);

        EXPECT_EQ(
            listener.get(),
            handler.getHandlerStore(tap::can::CanBus::CAN_BUS1)[normalizedId]);

        handler.removeReceiveHandler(*listener);

        EXPECT_EQ(nullptr, handler.getHandlerStore(tap::can::CanBus::CAN_BUS1)[normalizedId]);
    }
}

TEST_F(CanRxHandlerTest, MessageIsProcessedByCorrectListener)
{
    constructListeners();

    for (auto &listener : listeners)
    {
        EXPECT_CALL(*listener, processMessage);
    }

    for (auto &listener : listeners)
    {
        handler.attachReceiveHandler(listener.get());

        const modm::can::Message rxMessage(listener->canIdentifier);

        handler.processReceivedCanData(
            rxMessage,
            handler.getHandlerStore(tap::can::CanBus::CAN_BUS1));
    }
}

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
