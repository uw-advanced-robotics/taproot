#ifndef BMI088_HPP_
#define BMI088_HPP_

#include "tap/algorithms/MahonyAHRS.h"

#include "modm/processing/protothread.hpp"

#include "bmi088_register_table.hpp"

/*
acc requires further steps for initialization when using spi
*/

namespace sensors::bmi088
{
class Bmi088 : public ::modm::pt::Protothread
{
public:
    void init();
    bool run();

private:
    uint8_t tx, rx;

    Mahony mahonyAlgorithm;

    // modm::ResumableResult<bool> readGyroData();
    // modm::ResumableResult<bool> readAccData();
};

}  // namespace sensors::bmi088

#endif  // BMI088_HPP_
