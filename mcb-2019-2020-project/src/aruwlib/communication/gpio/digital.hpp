#ifndef DIGITAL_HPP_
#define DIGITAL_HPP_

#ifndef ENV_SIMULATOR
#include <modm/platform.hpp>
#endif

#include <stdint.h>

namespace aruwlib
{
namespace gpio
{
/**
 * Similar to the Analog class, wraps input and output digital pins.
 * Currently 4 input and 4 output pins are configured.
 *
 * @see InputPin for the input pins configured (pin names correspond
 *      to RoboMaster type A board definitions).
 * @see OutputPin for the output pins configured (pin names correspond
 *      to RoboMaster type A board definitions).
 */
class Digital
{
public:
    Digital() = default;
    Digital(const Digital &) = delete;
    Digital &operator=(const Digital &) = default;

    ///< Currently enabled digital input pins.
    enum InputPin
    {
        A,
        B,
        C,
        D
    };

    ///< Currently enabled digital output pins.
    enum OutputPin
    {
        E,
        F,
        G,
        H
    };

#ifdef ENV_SIMULATOR
    enum InputPullMode
    {
        Floating,
        PullUp,
        PullDown
    };
#else
    ///< This references a struct defined by modm.  Can either be floating, pull-up, or pull-down.
    using InputPullMode = modm::platform::Gpio::InputType;
#endif

    /**
     * Initializes all pins as output/input pins. Does not handle configuring
     * pin types (@see configureInputPullMode).
     */
    void init();

    /**
     * By default input pins are floating. Configure them to have a pull-up
     * or pull-down resistor here.
     *
     * @param[in] pin the InputPin to configure.
     * @param[in] mode the pull mode to be enabled.
     */
    void configureInputPullMode(InputPin pin, InputPullMode mode);

    /**
     * Sets the digital OutputPin either high or low.
     *
     * @param[in] pin the OutputPin to set.
     * @param[in] isSet `true` to send high, `false` to send low.
     */
    void set(OutputPin pin, bool isSet);

    /**
     * Reads from an InputPin.
     *
     * @param[in] pin the InputPin to read from.
     * @return `true` if the pin is pulled high and `false` otherwise.
     */
    bool read(InputPin pin) const;
};  // class Digital

}  // namespace gpio

}  // namespace aruwlib

#endif  // DIGITAL_HPP_
