#ifndef KEY_FORWARD_ANALOG_REMOTE_MAPPING
#define KEY_FORWARD_ANALOG_REMOTE_MAPPING


#include "analog_remote_mapping.hpp"
#include "tap/communication/serial/remote.hpp"


namespace tap::control
{


class KeyForwardAnalogRemoteMapping : public AnalogRemoteMapping
{
public:
    inline KeyForwardAnalogRemoteMapping(const tap::communication::serial::Remote& remote) : remote(remote) {}
    inline float getMax() const { return 1.0f; }
    inline float getMin() const { return -1.0f; }
    inline float getValue() const
    {
        if (remote.keyPressed(tap::communication::serial::Remote::Key::W) && !remote.keyPressed(tap::communication::serial::Remote::Key::S))
        {
            return 1.0;
        }
        else if (remote.keyPressed(tap::communication::serial::Remote::Key::S) && !remote.keyPressed(tap::communication::serial::Remote::Key::W))
        {
            return -1.0;
        }
        else
        {
            return 0.0;
        }
    }
private:
    const tap::communication::serial::Remote& remote;
};


class KeySidewaysAnalogRemoteMapping : public AnalogRemoteMapping
{
public:
    inline KeySidewaysAnalogRemoteMapping(const tap::communication::serial::Remote& remote) : remote(remote) {}
    inline float getMax() const { return 1.0f; }
    inline float getMin() const { return -1.0f; }
    inline float getValue() const
    {
        if (remote.keyPressed(tap::communication::serial::Remote::Key::D) && !remote.keyPressed(tap::communication::serial::Remote::Key::A))
        {
            return 1.0;
        }
        else if (remote.keyPressed(tap::communication::serial::Remote::Key::A) && !remote.keyPressed(tap::communication::serial::Remote::Key::D))
        {
            return -1.0;
        }
        else
        {
            return 0.0;
        }
    }
private:
    const tap::communication::serial::Remote& remote;
};

}


#endif