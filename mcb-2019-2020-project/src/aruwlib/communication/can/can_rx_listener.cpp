#include "can_rx_listener.hpp"

#include "aruwlib/Drivers.hpp"

namespace aruwlib
{
namespace can
{
CanRxListener::CanRxListener(uint32_t id, CanBus cB) : canIdentifier(id), canBus(cB)
{
    Drivers::canRxHandler.attachReceiveHandler(this);
}

CanRxListener::~CanRxListener() { Drivers::canRxHandler.removeReceiveHandler(*this); }
}  // namespace can

}  // namespace aruwlib
