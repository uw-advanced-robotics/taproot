#ifndef DISTANCE_SENSOR_H_
#define DISTANCE_SENSOR_H_

#include "aruwlib/rm-dev-board-a/board.hpp"

namespace aruwlib {

namespace sensors {

class DistanceSensor {
 public:
    /**
     * Constructor to init boundaries.
     * 
     * @param[in] minDistance the min valid distance.
     * @param[in] maxDistance the max valid distance.
     */
    DistanceSensor(float minDistance, float maxDistance);

    ///< Default destructor
    virtual ~DistanceSensor() = default;

    ///< Initialize the sensor.
    virtual void init() = 0;

    ///< Read sensor and updates current distance.
    virtual float read() = 0;

    ///< Checks if current reading is within bounds.
    virtual bool validReading() const = 0;

    ///< Get minumum distance boundary.
    float getMinDistance() const;

    ///< Get maximun distance boundary.
    float getMaxDistance() const;

    ///< Get the current distance.
    float getDistance() const;

 protected:
    // Distance from sensor
    float distance;

    // Lower boundary for reliable readings
    float minDistance;

    // Upper boundary for reliable readings
    float maxDistance;
};  // class DistanceSensor

}  // namespace sensors

}  // namespace aruwlib

#endif  // DISTANCE_SENSOR_H_
