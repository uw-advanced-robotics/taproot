#ifndef ANALOG_INPUT_HPP_
#define ANALOG_INPUT_HPP_


#include "tap/communication/serial/remote.hpp"


namespace tap::control::command_mapping
{

class AnalogInput
{
public:
    virtual float getMax() const = 0;
    virtual float getMin() const = 0;
    virtual float getValue(tap::communication::serial::Remote& remote) const = 0;
};

}

#endif  // ANALOG_INPUT_HPP_