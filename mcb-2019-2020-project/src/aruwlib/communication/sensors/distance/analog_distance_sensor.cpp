#include "analog_distance_sensor.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init analog IR boundary, distance conversion, and analog pin
    AnalogDistanceSensor::AnalogDistanceSensor(
        float minDistance,
        float maxDistance,
        float m,
        float b,
        float offset,
        gpio::Analog::Pin pin):
        DistanceSensor(minDistance, maxDistance),
        m(m),
        b(b),
        offset(offset),
        pin(pin) {}

    // Read sensor and update current distance
    float AnalogDistanceSensor::read() {
        // Read analog pin and convert to volts
        float reading = aruwlib::gpio::Analog::Read(pin);

        // Linear model
        float linear = m * reading / 1000.0 + b;

        // Convert to cm distance
        distance = 1 / linear + offset;

        return validReading() ? distance : -1.0f;
    }

    // Checks if current reading is within bounds
    bool AnalogDistanceSensor::validReading() {
        return (distance > minDistance) && (distance < maxDistance);
    }
}  // namespace sensors

}  // namespace aruwlib
