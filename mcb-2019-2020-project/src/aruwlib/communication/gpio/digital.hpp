#ifndef DIGITAL_HPP
#define DIGITAL_HPP

#ifndef ENV_SIMULATOR
#include <modm/platform.hpp>
#endif

#include <stdint.h>

namespace aruwlib
{

namespace gpio
{

class Digital {
 public:
    enum InputPin
    {
        A, B, C, D
    };

    enum OutputPin
    {
        E, F, G, H
    };

    #ifdef ENV_SIMULATOR
    enum InputPullMode
    {
        Floating, PullUp, PullDown
    };
    #else
    using InputPullMode = modm::platform::Gpio::InputType;
    #endif

    static void init();

    static void configureInputPullMode(InputPin pin, InputPullMode mode);

    static void set(OutputPin pin, bool isSet);

    static bool read(InputPin pin);
};

}  // namespace gpio

}  // namespace aruwlib

#endif
