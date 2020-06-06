#ifndef __CAN_HPP__
#define __CAN_HPP__

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

class Can
{
public:
    Can() = default;
    Can(const Can &) = delete;
    Can &operator=(const Can &) = default;

    void initialize();

    bool isMessageAvailable(CanBus bus) const;
    bool getMessage(CanBus bus, modm::can::Message *message);

    bool isReadyToSend(CanBus bus) const;
    bool sendMessage(CanBus bus, const modm::can::Message &message);
};

}  // namespace can
}  // namespace aruwlib
#endif
