#include "analog_distance_sensor.hpp"

#include "aruwlib/Drivers.hpp"

namespace aruwlib
{
namespace sensors
{
AnalogDistanceSensor::AnalogDistanceSensor(
    float minDistance,
    float maxDistance,
    float m,
    float b,
    float offset,
    gpio::Analog::Pin pin)
    : DistanceSensor(minDistance, maxDistance),
      m(m),
      b(b),
      offset(offset),
      pin(pin)
{
}

float AnalogDistanceSensor::read()
{
    // Read analog pin and convert to volts
    float reading = Drivers::analog.read(pin);

    // Linear model
    float linear = m * reading / 1000.0f + b;

    // Convert to cm distance
    distance = 1.0f / linear + offset;

    return validReading() ? distance : -1.0f;
}

bool AnalogDistanceSensor::validReading() const
{
    return (distance > minDistance) && (distance < maxDistance);
}
}  // namespace sensors

}  // namespace aruwlib
