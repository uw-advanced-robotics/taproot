/*
 * This file is part of ARUW's repository.
 *
 * Interfaces with modm receive data from CAN1 and CAN2 buses.
 *
 * To use, extend CanRxListner class and create a method called
 * processMessage. Next, call the function attachReceiveHandler,
 * which will add the class you instantiated to a list of classes
 * that will be handled on receive. The class you created and
 * attached will be called by the pollCanData function
 * every time there is a message available that has the CAN identifier
 * matching the identifier specified in the CanRxListner constructor.
 *
 * For proper closed loop motor control, it is necessary to have the
 * pollCanData function be called at a very high frequency,
 * so call this in a high frequency thread.
 */

#ifndef __CAN_INTERFACE_HPP__
#define __CAN_INTERFACE_HPP__

#include "can_rx_listener.hpp"

namespace aruwlib
{
namespace can
{
class CanRxHandler
{
public:
    CanRxHandler() = default;
    CanRxHandler(const CanRxHandler&) = delete;
    CanRxHandler& operator=(const CanRxHandler&);

    // Call this function to add a CanRxListner to the list of CanRxListner's
    // that are referenced when a new CAN message is received.
    void attachReceiveHandler(CanRxListner* const f);

    // Function handles receiving messages and calling the appropriate
    // processMessage function given the CAN bus and can identifier.
    void pollCanData();

    void removeReceiveHandler(const CanRxListner& rxListner);

private:
    static const int MAX_RECEIVE_UNIQUE_HEADER_CAN1 = 8;
    static const int MAX_RECEIVE_UNIQUE_HEADER_CAN2 = 8;
    static const uint16_t LOWEST_RECEIVE_ID = 0x201;

    CanRxListner* messageHandlerStoreCan1[MAX_RECEIVE_UNIQUE_HEADER_CAN1] = {0};

    CanRxListner* messageHandlerStoreCan2[MAX_RECEIVE_UNIQUE_HEADER_CAN2] = {0};

    void attachReceiveHandler(
        CanRxListner* const CanRxHndl,
        CanRxListner** messageHandlerStore,
        int16_t messageHandlerStoreSize);

    void processReceivedCanData(
        const modm::can::Message& rxMessage,
        CanRxListner* const* messageHandlerStore);

    void remoteReceiveHandler(const CanRxListner& rxListner, CanRxListner** messageHandlerStore);
};

}  // namespace can

}  // namespace aruwlib

#endif
