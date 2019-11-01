#include "src/communication/can/can_rx_listener.hpp"
#include "src/communication/can/can_rx_handler.hpp"

namespace aruwlib
{

namespace can
{
    CanRxListner::~CanRxListner()
    {
        CanRxHandler::removeReceiveHandler(*this);
    }

    CanRxListner::CanRxListner(uint32_t id, CanBus cB) : canIdentifier(id), canBus(cB)
    {
        CanRxHandler::attachReceiveHandler(this);
    }
}  // namespace can

}  // namespace aruwlib
