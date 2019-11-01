#ifndef __CAN_RX_LISTENER__
#define __CAN_RX_LISTENER__

#include "rm-dev-board-a/board.hpp"

namespace aruwlib
{

namespace can
{

    // must declare a CAN bus when declaring a motor
    // either CAN 1 or CAN 2 line
    enum class
    CanBus
    {
        CAN_BUS1,
        CAN_BUS2,
    };

// You must extend this class in order to use the features in this
// namespace.
class CanRxListner
{
 public:
    // Construct a new CanRxListner, must specify the can identifier
    // and the can bus the receive handler will be watching.
    CanRxListner(uint32_t id, CanBus cB);

    // delete copy constructor
    CanRxListner(const CanRxListner&) = delete;

    // Remove from receive interface
    ~CanRxListner();

    // Overridden function: When you extend this class declare this
    // function.
    virtual void processMessage(const modm::can::Message& message) = 0;

    // Variables that are necessary for the receive handler to determine
    // which message corresponds to which CanRxListner child class.
    uint32_t canIdentifier;

    CanBus canBus;
};

}  // namespace can

}  // namespace aruwlib

#endif
