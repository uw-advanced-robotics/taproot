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
#include "tap/motor/dji_motor_ids.hpp"

using namespace testing;
using namespace std;
using namespace tap::mock;

class CanRxHandlerTest : public Test
{
protected:
    CanRxHandlerTest() : handler(&drivers) {}

    void constructListeners(tap::can::CanBus canBus = tap::can::CanBus::CAN_BUS1)
    {
        for (uint32_t i = tap::motor::MOTOR1; i <= tap::motor::MOTOR8; i++)
        {
            auto listener = make_unique<CanRxListenerMock>(&drivers, i, canBus);
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

TEST_F(CanRxHandlerTest, attachReceiveHandler_attaches_listener_can2)
{
    CanRxListenerMock listener(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS2);

    handler.attachReceiveHandler(&listener);

    EXPECT_EQ(
        &listener,
        handler.getHandlerStore(
            tap::can::CanBus::CAN_BUS2)[tap::can::CanRxHandler::binIndexForCanId(
            listener.canIdentifier)]);

    handler.removeReceiveHandler(listener);
}

TEST_F(CanRxHandlerTest, attach_and_remove_recieve_handler_with_bin_conflicts_in_order)
{
    CanRxListenerMock listener(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS2);
    CanRxListenerMock listener2(
        &drivers,
        tap::motor::MOTOR1 + tap::can::CanRxHandler::CAN_BINS,
        tap::can::CanBus::CAN_BUS2);

    handler.attachReceiveHandler(&listener);
    handler.attachReceiveHandler(&listener2);

    EXPECT_EQ(
        &listener,
        handler.getHandlerStore(
            tap::can::CanBus::CAN_BUS2)[tap::can::CanRxHandler::binIndexForCanId(
            listener.canIdentifier)]);

    EXPECT_EQ(
        &listener2,
        handler
            .getHandlerStore(tap::can::CanBus::CAN_BUS2)[tap::can::CanRxHandler::binIndexForCanId(
                listener.canIdentifier)]
            ->next);

    handler.removeReceiveHandler(listener);

    EXPECT_EQ(
        &listener2,
        handler.getHandlerStore(
            tap::can::CanBus::CAN_BUS2)[tap::can::CanRxHandler::binIndexForCanId(
            listener.canIdentifier)]);

    handler.removeReceiveHandler(listener2);
}

TEST_F(CanRxHandlerTest, attach_and_remove_recieve_handler_with_bin_conflicts_in_reverse_order)
{
    CanRxListenerMock listener(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS2);
    CanRxListenerMock listener2(
        &drivers,
        tap::motor::MOTOR1 + tap::can::CanRxHandler::CAN_BINS,
        tap::can::CanBus::CAN_BUS2);

    handler.attachReceiveHandler(&listener);
    handler.attachReceiveHandler(&listener2);

    EXPECT_EQ(
        &listener,
        handler.getHandlerStore(
            tap::can::CanBus::CAN_BUS2)[tap::can::CanRxHandler::binIndexForCanId(
            listener.canIdentifier)]);

    EXPECT_EQ(
        &listener2,
        handler
            .getHandlerStore(tap::can::CanBus::CAN_BUS2)[tap::can::CanRxHandler::binIndexForCanId(
                listener.canIdentifier)]
            ->next);

    handler.removeReceiveHandler(listener2);

    EXPECT_EQ(
        &listener,
        handler.getHandlerStore(
            tap::can::CanBus::CAN_BUS2)[tap::can::CanRxHandler::binIndexForCanId(
            listener.canIdentifier)]);

    EXPECT_EQ(
        nullptr,
        handler
            .getHandlerStore(tap::can::CanBus::CAN_BUS2)[tap::can::CanRxHandler::binIndexForCanId(
                listener.canIdentifier)]
            ->next);

    handler.removeReceiveHandler(listener);
}

TEST_F(CanRxHandlerTest, ListenerAttachesAndDetatchesInArray)
{
    constructListeners();

    for (auto &listener : listeners)
    {
        handler.attachReceiveHandler(listener.get());

        int bin = tap::can::CanRxHandler::binIndexForCanId(listener->canIdentifier);

        EXPECT_EQ(listener.get(), handler.getHandlerStore(tap::can::CanBus::CAN_BUS1)[bin]);

        handler.removeReceiveHandler(*listener);

        EXPECT_EQ(nullptr, handler.getHandlerStore(tap::can::CanBus::CAN_BUS1)[bin]);
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

TEST_F(CanRxHandlerTest, process_messages_with_bin_conflicts)
{
    CanRxListenerMock listener(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS1);
    CanRxListenerMock listener2(
        &drivers,
        tap::motor::MOTOR1 + tap::can::CanRxHandler::CAN_BINS,
        tap::can::CanBus::CAN_BUS1);

    handler.attachReceiveHandler(&listener);
    handler.attachReceiveHandler(&listener2);

    EXPECT_CALL(listener, processMessage);
    const modm::can::Message rxMessage(listener.canIdentifier);
    handler.processReceivedCanData(rxMessage, handler.getHandlerStore(tap::can::CanBus::CAN_BUS1));

    EXPECT_CALL(listener2, processMessage);
    const modm::can::Message rxMessage2(listener2.canIdentifier);
    handler.processReceivedCanData(rxMessage2, handler.getHandlerStore(tap::can::CanBus::CAN_BUS1));

    handler.removeReceiveHandler(listener);
    handler.removeReceiveHandler(listener2);
}

TEST_F(CanRxHandlerTest, attachReceiveHandler__error_logged_with_overloading_can_rx_listener_id)
{
    CanRxListenerMock canRxListener(&drivers, 0, tap::can::CanBus::CAN_BUS1);
    CanRxListenerMock canRxListener2(&drivers, 0, tap::can::CanBus::CAN_BUS1);

    handler.attachReceiveHandler(&canRxListener);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(1);
    handler.attachReceiveHandler(&canRxListener2);
    EXPECT_EQ(&canRxListener, handler.getHandlerStore(tap::can::CanBus::CAN_BUS1)[0]);

    handler.removeReceiveHandler(canRxListener);
}

TEST_F(
    CanRxHandlerTest,
    removeReceiveHandler__error_logged_with_missing_can_rx_listener_id_with_empty_bin)
{
    CanRxListenerMock canRxListener(&drivers, 0, tap::can::CanBus::CAN_BUS1);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(1);

    handler.removeReceiveHandler(canRxListener);
}

TEST_F(
    CanRxHandlerTest,
    removeReceiveHandler__error_logged_with_missing_can_rx_listener_id_with_existing_bin)
{
    CanRxListenerMock canRxListener(&drivers, 0, tap::can::CanBus::CAN_BUS1);
    CanRxListenerMock canRxListener2(
        &drivers,
        tap::can::CanRxHandler::CAN_BINS,
        tap::can::CanBus::CAN_BUS1);

    handler.attachReceiveHandler(&canRxListener);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(1);
    handler.removeReceiveHandler(canRxListener2);

    handler.removeReceiveHandler(canRxListener);
}

TEST_F(CanRxHandlerTest, pollCanData_can1_calls_process_message_passing_msg_to_correct_listener)
{
    constructListeners();

    handler.attachReceiveHandler(listeners[0].get());

    modm::can::Message msg(tap::motor::MOTOR1, 8, 0xffff'ffff'ffff'ffff, false);

    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS1, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message *message) {
            *message = msg;
            return true;
        });
    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS2, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message *) { return false; });

    EXPECT_CALL(*listeners[0], processMessage);

    handler.pollCanData();
}

TEST_F(CanRxHandlerTest, pollCanData_can2_calls_process_message_passing_msg_to_correct_listener)
{
    constructListeners(tap::can::CanBus::CAN_BUS2);

    handler.attachReceiveHandler(listeners[0].get());

    modm::can::Message msg(tap::motor::MOTOR1, 8, 0xffff'ffff'ffff'ffff, false);

    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS1, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message *) { return false; });
    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS2, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message *message) {
            *message = msg;
            return true;
        });

    EXPECT_CALL(*listeners[0], processMessage);

    handler.pollCanData();
}
