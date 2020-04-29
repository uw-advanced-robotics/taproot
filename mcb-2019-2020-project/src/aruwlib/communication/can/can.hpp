#ifndef __CAN_HPP__
#define __CAN_HPP__

#include <modm/architecture/interface/can_message.hpp>

namespace aruwlib
{

namespace can
{
enum class
CanBus
{
    CAN_BUS1,
    CAN_BUS2,
};

class Can {
 public:
    static void initialize();

    static bool isMessageAvailable(CanBus bus);
    static bool getMessage(CanBus bus, modm::can::Message *message);

    static bool isReadyToSend(CanBus bus);
    static bool sendMessage(CanBus bus, const modm::can::Message& message);
};

}  // namespace can
}  // namespace aruwlib
#endif
