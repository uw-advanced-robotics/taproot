#ifndef LEDS_HPP_
#define LEDS_HPP_

#include <stdint.h>

namespace aruwlib
{

namespace gpio
{

/**
 * A class specifically for handling the wrapping of the digital
 * pins connected to LEDs on the RoboMaster type A board.
 */
class Leds {
 public:
    /**
     * The LED letters correspond to the letters written next to the LEDs
     * on the RoboMaster type A board. The `Green` LED is the led next
     * to the button (opposite from the 8 LEDs labeled by letters).  The
     * `Red` LED is the led directly adjacent to the `Green` LED. A third
     * red LED is triggered when you supply 24V to the board and is not
     * controllable.
     */
    enum LedPin
    {
        A = 0, B, C, D, E, F, G, H, Green, Red
    };

    /**
     * Initializes the LEDs by putting the pins in output mode and settting
     * all the pins to low.
     */
    static void init();

    /**
     * Sets a given led to either high or low.
     * 
     * @note setting an LED to high (`isSet=true`) will turn the LED off and setting
     *      an LED to low (`isSet=false`) will turn the LED on.
     * @param[in] pin the LED to set
     * @param[in] isSet `true` if you want to turn the LED off, `false` if you want
     *      to turn the LED on.
     */
    static void set(LedPin pin, bool isSet);
};  // class Leds

}  // namespace gpio

}  // namespace aruwlib

#endif  // LEDS_HPP_
