/*
 * This file is part of ARUW's repository
 * 
 * Analog output pins are pins W, X, Y, and Z (board pins PI5, PI6, PI7, PI2)
 * 
 * to read from a pin call Read and pass the function a value (S - V) from the 
 * analog inPin enum. 
 */

#ifndef ANALOG_HPP
#define ANALOG_HPP

#include <modm/platform/adc/adc_1.hpp>

namespace aruwlib
{

namespace gpio
{

class Analog {
 public:
    // Analog pins
    enum Pin
    {
        S = 1, T, U, V
    };

    static void init();

    // Reads voltage across the specified pin. Units in mV.
    static uint16_t Read(Analog::Pin pin);
};

}  // namespace gpio

}  // namespace aruwlib

#endif
