#ifndef LEDS_HPP
#define LEDS_HPP

#include <stdint.h>

namespace aruwlib
{

namespace gpio
{

class Leds {
 public:
    enum LedPin
    {
        A = 0, B, C, D, E, F, G, H, Green, Red
    };

    static void init();

    static void set(LedPin pin, bool isSet);
};

}  // namespace gpio

}  // namespace aruwlib

#endif
