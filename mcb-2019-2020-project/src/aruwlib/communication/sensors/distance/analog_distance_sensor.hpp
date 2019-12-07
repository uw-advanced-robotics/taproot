/*
Basic analog IR Sensor
The distance conversion can be tweaked depending on the sensor
min and max distance are in cm
*/

#ifndef ANALOG_DISTANCE_SENSOR_H
#define ANALOG_DISTANCE_SENSOR_H

#include "distance_sensor.hpp"
#include "src/aruwlib/communication/gpio/analog.hpp"

namespace aruwlib {

namespace sensors {

class AnalogDistanceSensor: public DistanceSensor {
 public:
    // Constructor to init analog IR boundary, distance conversion, and analog pin
    AnalogDistanceSensor(
        float minDistance,
        float maxDistance,
        float m,
        float b,
        float offset,
        gpio::Analog::Pin pin
    );

    // Read sensor and updates current distance
    float read(void) override;

    // Checks if current reading is within bounds
    bool validReading(void) override;

 private:
    // Distance calulation values for linear model y=mx+b
    float m;
    float b;

    // Offset value of inverse
    float offset;

    // Analog pin
    gpio::Analog::Pin pin;
};

}  // namespace sensors

}  // namespace aruwlib

#endif  // ANALOG_DISTANCE_SENSOR_H
