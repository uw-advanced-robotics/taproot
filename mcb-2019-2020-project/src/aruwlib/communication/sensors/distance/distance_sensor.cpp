#include "distance_sensor.hpp"

namespace aruwlib
{
namespace sensors
{
// Constructor to init boundaries
DistanceSensor::DistanceSensor(float minDistance, float maxDistance)
    : distance(0),
      minDistance(minDistance),
      maxDistance(maxDistance)
{
}

float DistanceSensor::getMinDistance() const { return minDistance; }

float DistanceSensor::getMaxDistance() const { return maxDistance; }

float DistanceSensor::getDistance() const { return distance; }

}  // namespace sensors

}  // namespace aruwlib
