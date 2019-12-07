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

#include <modm/platform/can/can_1.hpp>
#include <modm/platform/can/can_2.hpp>
#include "can_rx_listener.hpp"

namespace aruwlib
{

namespace can
{

class CanRxHandler
{
 public:
    // delete copy constructor
    CanRxHandler(const CanRxHandler&) = delete;

    // Call this function to add a CanRxListner to the list of CanRxListner's
    // that are referenced when a new CAN message is received.
    static void attachReceiveHandler(CanRxListner*const f);

    // Function handles receiving messages and calling the appropriate
    // processMessage function given the CAN bus and can identifier.
    static void pollCanData(void);

    static void removeReceiveHandler(const CanRxListner& rxListner);

 private:
    #define MAX_RECEIVE_UNIQUE_HEADER_CAN1 8
    #define MAX_RECEIVE_UNIQUE_HEADER_CAN2 8
    #define LOWEST_RECEIVE_ID (0x201)

    static CanRxListner* messageHandlerStoreCan1[MAX_RECEIVE_UNIQUE_HEADER_CAN1];

    static CanRxListner* messageHandlerStoreCan2[MAX_RECEIVE_UNIQUE_HEADER_CAN2];

    static void attachReceiveHandler(
        CanRxListner *const CanRxHndl,
        CanRxListner** messageHandlerStore,
        int16_t messageHandlerStoreSize
    );

    static void processReceivedCanData(
        const modm::can::Message& rxMessage,
        CanRxListner *const* messageHandlerStore,
        const bool messageAvailable
    );

    static void remoteReceiveHandler(
        const CanRxListner& rxListner,
        CanRxListner** messageHandlerStore
    );
};

}  // namespace can

}  // namespace aruwlib

#endif
