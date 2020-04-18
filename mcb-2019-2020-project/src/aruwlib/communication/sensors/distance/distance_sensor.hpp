/*
General distance sensor class for basic analog IR or other sensors to build off of
Planned sensors: basic analog IR, Sharp IR, Adafruit VL6180X, and Seeed IR
*/

#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "aruwlib/rm-dev-board-a/board.hpp"

namespace aruwlib {

namespace sensors {

class DistanceSensor {
 public:
    // Constructor to init boundaries
    DistanceSensor(float minDistance, float maxDistance);

    // Destructor
    virtual ~DistanceSensor(void) = 0;

    // Init sensor
    virtual void init(void) = 0;

    // Read sensor and updates current distance
    virtual float read(void) = 0;

    // Checks if current reading is within bounds
    virtual bool validReading(void) = 0;

    // Get minumum distance boundary
    float getMinDistance(void);

    // Get maximun distance boundary
    float getMaxDistance(void);

 protected:
    // Distance from sensor
    float distance;

    // Lower boundary for reliable readings
    float minDistance;

    // Upper boundary for reliable readings
    float maxDistance;
};

}  // namespace sensors

}  // namespace aruwlib

#endif  // DISTANCE_SENSOR_H
