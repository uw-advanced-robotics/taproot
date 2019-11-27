/*
Basic SHARP IR Sensor via analog input
The distance conversion can be tweaked depending on the sensor
*/

#ifndef SHARP_IR_H
#define SHARP_IR_H

#include "analog_distance_sensor.hpp"

namespace aruwlib {

namespace sensors {

class SharpIrGP2Y0A41: public AnalogDistanceSensor {
 public:
    // Constructor to init Sharp IR analog pin
    explicit SharpIrGP2Y0A41(gpio::Analog::Pin pin);

    // Init not needed for Sharp IR
    void init() override {}

 private:
    // Distance boundary values
    #define SHARP_IR_MIN 4.0f
    #define SHARP_IR_MAX 30.0f

    // Distance calculation values for SHARP 0A41SK F IR Sensor
    // Subject to change
    #define SHARP_IR_M 0.072
    #define SHARP_IR_B -0.008
    #define SHARP_IR_OFFSET -0.42
};

}  // namespace sensors

}  // namespace aruwlib

#endif  // SHARP_IR_H
