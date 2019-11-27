#include "distance_sensor.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init boundaries
    DistanceSensor::DistanceSensor(float minDistance, float maxDistance):
        distance(0),
        minDistance(minDistance),
        maxDistance(maxDistance) {}

    // Destructor
    DistanceSensor::~DistanceSensor() {}

    // Get minumum distance boundary
    // cppcheck-suppress unusedFunction //TODO Remove lint suppression
    float DistanceSensor::getMinDistance() {
        return minDistance;
    }

    // Get maximun distance boundary
    // cppcheck-suppress unusedFunction //TODO Remove lint suppression
    float DistanceSensor::getMaxDistance() {
        return maxDistance;
    }
}  // namespace sensors

}  // namespace aruwlib
