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
class Bmi088 : public ::modm::pt::Protothread, modm::Resumable<2>
{
public:
    void init();
    bool run();

private:
    uint8_t tx, rx;

    struct GyroData
    {
        int16_t gx, gy, gz;
    } rawGyro;

    struct AccData
    {
        int16_t ax, ay, az;
    } rawAcc;

    Mahony mahonyAlgorithm;

    modm::ResumableResult<bool> readGyroData();
    modm::ResumableResult<bool> readAccData();
};

}  // namespace sensors::bmi088

#endif  // BMI088_HPP_
