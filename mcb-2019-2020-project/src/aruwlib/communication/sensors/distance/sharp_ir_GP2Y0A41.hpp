#ifndef SHARP_IR_HPP_
#define SHARP_IR_HPP_

#include "analog_distance_sensor.hpp"

namespace aruwlib {

namespace sensors {

/**
 * Basic SHARP IR Sensor via analog input.  Returns distance in cm.
 *
 * The distance conversion can be tweaked depending on the sensor.
 */
class SharpIrGP2Y0A41: public AnalogDistanceSensor {
 public:
    /**
     * Constructor to init Sharp IR analog pin.
     *
     * @param[in] pin the analog pin to attach the sensor to.
     */
    explicit SharpIrGP2Y0A41(gpio::Analog::Pin pin);

    ///< Init not needed for Sharp IR.
    void init() override {}

 private:
    ///< Min distance boundary value (in cm).
    static constexpr float SHARP_IR_MIN = 4.0f;
    ///< Max distance boundary value (in cm).
    static constexpr float SHARP_IR_MAX = 30.0f;

    // Subject to change.
    ///< Distance calculation values for SHARP 0A41SK F IR Sensor.
    static constexpr float SHARP_IR_M = 0.072f;
    ///< Distance calculation values for SHARP 0A41SK F IR Sensor.
    static constexpr float SHARP_IR_B = -0.008f;
    ///< Distance calculation values for SHARP 0A41SK F IR Sensor.
    static constexpr float SHARP_IR_OFFSET = -0.42f;
};  // class SharpIrGP2Y0A41

}  // namespace sensors

}  // namespace aruwlib

#endif  // SHARP_IR_HPP_
