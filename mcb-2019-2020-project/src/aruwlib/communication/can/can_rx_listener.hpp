#ifndef __CAN_RX_LISTENER__
#define __CAN_RX_LISTENER__

#include <cstdint>

#include <modm/architecture/interface/can_message.hpp>

#include "can.hpp"

namespace aruwlib
{
namespace can
{
/**
 * A class that when extended allows you to interface with the `can_rx_handler`.
 *
 * You must extend this class in order to use its features. So to use,
 * extend this class and implement `processMessage`. This pure virtual
 * function will be called in `can_rx_handler`'s processReceivedCanData.
 * Additionally, you must call `initialize`.
 *
 * Below is a barebones example implementation of a `CanRxListener`.
 *
 * ```
 * class CanWatcher : public CanRxListener
 * {
 *  public:
 *     CanWatcher(uint32_t id, CanBus cB) : CanRxListener(id, cB) {}
 *
 *     void processMessage(const modm::can::Message& message) override
 *     {
 *          myMessage = message;
 *     }
 *
 *  private:
 *     modm::can::Message myMessage;
 * };
 * ```
 *
 * Then in `main.cpp`.
 *
 * ```
 * CanWatcher cw;
 *
 * int main(int argv, char **argc)
 * {
 *     while (true)
 *     {
 *         Drivers::canRxHandler.pollCanData();
 *     }
 * }
 * ```
 *
 * @see `CanRxHandler`
 * @see `DjiMotor` for a usecase implementation of a `CanRxListener`.
 */
class CanRxListener
{
public:
    /**
     * Construct a new CanRxListener, must specify the can identifier
     * and the can bus the receive handler will be watching. In doing so,
     * adds itself to the CanRxHandler.
     *
     * @param[in] id the message identifier to be associated with this
     *      rx listener.
     * @param[in] cB the CanBus that you would like to watch.
     */
    CanRxListener(uint32_t id, CanBus cB);

    ///< Delete copy constructor.
    CanRxListener(const CanRxListener&) = delete;

    ///< Delete operator=.
    CanRxListener& operator=(const CanRxListener& other) = delete;

    ///< Here we remove the listener from receive interface.
    ~CanRxListener();

    /**
     * Called when a message is received with the particular id and
     * CAN bus.
     *
     * Pure virtual function: When you extend this class declare this
     * function.
     *
     * @param[in] message a new message received on the CAN bus.
     */
    virtual void processMessage(const modm::can::Message& message) = 0;

    /**
     * A variable necessary for the receive handler to determine
     * which message corresponds to which CanRxListener child class.
     */
    const uint32_t canIdentifier;

    /**
     * A variable necessary for the receive handler to determine
     * which message corresponds to which CanRxListener child class.
     */
    const CanBus canBus;
};  // class CanRxListener

}  // namespace can

}  // namespace aruwlib

#endif
