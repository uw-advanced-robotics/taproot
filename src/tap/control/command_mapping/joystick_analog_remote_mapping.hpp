#ifndef KEY_FORWARD_ANALOG_REMOTE_MAPPING
#define KEY_FORWARD_ANALOG_REMOTE_MAPPING


#include "analog_remote_mapping.hpp"
#include "tap/communication/serial/remote.hpp"


namespace tap::control
{


class JoystickAnalogRemoteMapping : public AnalogRemoteMapping
{
public:
    inline JoystickAnalogRemoteMapping(const tap::communication::serial::Remote& remote, const tap::communication::serial::Remote::Channel channel) : remote(remote), channel(channel) {}
    inline float getValue() const
    {
        return remote.getChannel(channel);
    }
private:
    const tap::communication::serial::Remote& remote;
    const tap::communication::serial::Remote::Channel channel;
};

}


#endif