#ifndef CAN_HPP_
#define CAN_HPP_

#include <modm/architecture/interface/can_message.hpp>

namespace aruwlib
{
namespace can
{
enum class CanBus
{
    CAN_BUS1,
    CAN_BUS2,
};

/**
 * A simple CAN wrapper class that handles I/O from both CAN bus 1 and 2.
 */
class Can
{
public:
    Can() = default;
    Can(const Can &) = delete;
    Can &operator=(const Can &) = default;

    /**
     * Initializes CAN 1 and CAN 2 hardware to pins specific to the Robomaster
     * type A board and sets up the CAN bus filters necessary for reading
     * from the CAN bus.
     *
     * @note CAN 1 is connected to pins D0 (RX) and D1 (TX) and
     *      CAN 2 is connected to pins B12 (RX) and B12 (TX).
     * @note The CAN filters are set up to receive NOT extended identifier IDs.
     */
    void initialize();

    /**
     * Checks the passed in CanBus to see if there is a message waiting
     * and available.
     *
     * @param[in] bus the CanBus to check for a message.
     * @return true if a message is available, false otherwise.
     */
    bool isMessageAvailable(CanBus bus) const;

    /**
     * Checks the CanBus for a message and if a message is successfully
     * acquired, returns true and places the message in the return parameter
     * message.
     *
     * @param[in] bus the CanBus to acquire a message from.
     * @param[out] message a return parameter which the message is
     *      placed in.
     * @return true if a valid message was placed in the parameter
     *      message. False otherwise.
     */
    bool getMessage(CanBus bus, modm::can::Message *message);

    /**
     * Checks the given CanBus to see if the CanBus is idle.
     *
     * @param[in] bus the CanBus to check.
     * @return true if the bus is not busy, false otherwise.
     */
    bool isReadyToSend(CanBus bus) const;

    /**
     * Sends the passed in message over the CanBus. Returns whether or
     * not the message succeeded.
     *
     * @attention `modm::can::Message` defaults to an extended
     * message identifier. For all RoboMaster products we have, we do not
     * want our messages to be extended. Be sure to be explicit
     * when instantiating a message object and setting extended to
     * false.
     *
     * @param[in] bus the `CanBus` for which the message should be sent across.
     * @param[in] message the message to send
     * @return true if the message was successfully sent, false otherwise.
     */
    bool sendMessage(CanBus bus, const modm::can::Message &message);
};  // class Can

}  // namespace can
}  // namespace aruwlib

#endif  // CAN_HPP_