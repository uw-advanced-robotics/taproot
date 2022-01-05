#ifndef BMI088_HPP_
#define BMI088_HPP_

#include "tap/algorithms/MahonyAHRS.h"

#include "tap/communication/sensors/imu_heater/imu_heater.hpp"
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
class Bmi088 : private ::modm::pt::Protothread, public Bmi088Data
{
public:
    /**
     * Bit appended or removed from a register while reading/writing.
     */
    static constexpr uint8_t BMI088_READ_BIT = 0x80;
    static constexpr uint32_t BMI088_COMM_WAIT_SENSOR_TIME = 150;
    static constexpr uint32_t BMI088_COMM_LONG_WAIT_TIME = 80;

    static constexpr float BMI088_TEMP_FACTOR = 0.125f;
    static constexpr float BMI088_TEMP_OFFSET = 23.0f;

    static constexpr float BMI088_GYRO_2000_SEN = 0.00106526443603169529841533860381f;
    static constexpr float BMI088_ACCEL_3G_SEN = 0.0008974358974f;

    Bmi088(tap::Drivers *drivers);

    void initiailze();

    bool run();

    void periodicIMUUpdate();

    bool isReady() const { return ready; }

    float getYaw() { return mahonyAlgorithm.getYaw(); }
    float getPitch() { return mahonyAlgorithm.getPitch(); }
    float getRoll() { return mahonyAlgorithm.getRoll(); }

    int16_t getGxRaw() const { return data.gyro.x; }
    int16_t getGyRaw() const { return data.gyro.y;}
    int16_t getGzRaw() const { return data.gyro.z;}

    int16_t getAxRaw() const { return data.acc.x; }
    int16_t getAyRaw() const { return data.acc.y; }
    int16_t getAzRaw() const { return data.acc.z; }

    float getGx() const { return data.gyro.x * BMI088_GYRO_2000_SEN;  }
    float getGy() const { return data.gyro.y * BMI088_GYRO_2000_SEN;  }
    float getGz() const { return data.gyro.z * BMI088_GYRO_2000_SEN;  }

    float getAx() const { return data.acc.x * BMI088_ACCEL_3G_SEN; }
    float getAy() const { return data.acc.y * BMI088_ACCEL_3G_SEN; }
    float getAz() const { return data.acc.z * BMI088_ACCEL_3G_SEN; }

private:
    struct
    {
        struct
        {
            float x, y, z;
        } acc;

        struct
        {
            float x, y, z;
        } gyro;

        struct
        {
            float x, y, z;
        } accCalib;

        struct
        {
            float x, y, z;
        } gyroCalib;

        float temperature;
    } data;

    tap::Drivers *drivers;

    bool ready = false;

    // uint8_t tx, rx;

    Mahony mahonyAlgorithm;

    sensors::ImuHeater imuHeater;

    void initializeAcc();
    void initializeGyro();
};

}  // namespace tap::sensors::bmi088

#endif  // BMI088_HPP_
