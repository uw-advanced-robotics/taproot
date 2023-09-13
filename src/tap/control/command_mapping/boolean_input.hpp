#ifndef BOOLEAN_INPUT_HPP_
#define BOOLEAN_INPUT_HPP_

#include "tap/communication/serial/remote.hpp"

namespace tap::control::command_mapping
{

class BooleanInput
{
public:
    virtual bool triggered(tap::communication::serial::Remote& remote) const = 0;
};

}

#endif  // BOOLEAN_INPUT_HPP_