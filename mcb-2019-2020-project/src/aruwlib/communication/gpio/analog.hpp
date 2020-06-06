#ifndef ANALOG_HPP_
#define ANALOG_HPP_

#include <stdint.h>

#ifndef ENV_SIMULATOR
#include <modm/platform/adc/adc_1.hpp>
#endif

namespace aruwlib
{

namespace gpio
{

/**
 * Analog output pins are pins W, X, Y, and Z (board pins PI5, PI6, PI7, PI2)
 * as referenced by the pin naming on the RoboMaster type A board.
 * 
 * To read from a pin call Read and pass the function a value (S - V) from the 
 * analog inPin enum.
 */
class Analog {
 public:
    Analog() = default;
    Analog(const Analog&) = delete;
    Analog &operator=(const Analog&) = default;

    // Analog pins
    enum Pin
    {
        S = 1, T, U, V
    };

    ///< Initializes the ADC and connects the configured analog pins to it.
    void init();

    ///< Reads voltage across the specified pin. Units in mV.
    uint16_t read(Analog::Pin pin) const;
};  // class Analog
}  // namespace gpio

}  // namespace aruwlib

#endif  // ANALOG_HPP_
