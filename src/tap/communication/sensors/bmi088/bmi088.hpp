#ifndef BMI088_HPP_
#define BMI088_HPP_

#include "tap/algorithms/MahonyAHRS.h"

#include "modm/processing/protothread.hpp"

#include "bmi088_register_table.hpp"

namespace tap
{
class Drivers;
}

/*
acc requires further steps for initialization when using spi
*/

namespace tap::sensors::bmi088
{
class Bmi088 : private ::modm::pt::Protothread, private Bmi088Data
{
public:
    /**
     * Bit appended or removed from a register while reading/writing.
     */
    static constexpr uint8_t BMI088_READ_BIT = 0x80;
    static constexpr uint32_t BMI088_COMM_WAIT_SENSOR_TIME = 150;
    static constexpr uint32_t BMI088_COMM_LONG_WAIT_TIME = 80;

    void initiailze();

    bool run();

    void periodicIMUUpdate();

private:
    tap::Drivers *drivers;

    uint8_t tx, rx;

    Mahony mahonyAlgorithm;

    void initializeAcc();
    void initializeGyro();
};

}  // namespace sensors::bmi088

#endif  // BMI088_HPP_
