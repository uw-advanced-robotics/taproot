/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef BMI088_HPP_
#define BMI088_HPP_

#include "tap/algorithms/MahonyAHRS.h"

#include "tap/communication/sensors/imu_heater/imu_heater.hpp"
#include "modm/processing/protothread.hpp"

#include "bmi088_data.hpp"

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
    enum class ImuState
    {
        IMU_NOT_CONNECTED,
        IMU_NOT_CALIBRATED,
        IMU_CALIBRATING,
        IMU_CALIBRATED,
    };

    static constexpr float BMI088_TEMP_FACTOR = 0.125f;
    static constexpr float BMI088_TEMP_OFFSET = 23.0f;

    //TODO
    static constexpr float BMI088_GYRO_2000_SEN = 0.00106526443603169529841533860381f;
    static constexpr float BMI088_ACCEL_3G_SEN = 0.0008974358974f;

    static constexpr float ACCELERATION_SENSITIVITY = 0;//TODO

    /**
     * The number of samples we take in order to determine the mpu offsets.
     */
    static constexpr float BMI088_OFFSET_SAMPLES = 1000;

    Bmi088(tap::Drivers *drivers);

    void initiailze();

    bool run();

    void periodicIMUUpdate();

    /**
     * Returns the state of the IMU. Can be not connected, connected but not calibrated, or
     * calibrated. When not connected, IMU data will be garbage. When not calibrated, IMU data is
     * valid but the computed yaw angle data will drift. When calibrating, the IMU data is invalid.
     * When calibrated, the IMU data is valid and assuming proper calibration the IMU data should not
     * drift.
     *
     * To be safe, whenever you call the functions below, call this function to ensure
     * the data you are about to receive is not garbage.
     */
    ImuState getImuState() const;

    void requestRecalibration();

    float getYaw() { return mahonyAlgorithm.getYaw(); }
    float getPitch() { return mahonyAlgorithm.getPitch(); }
    float getRoll() { return mahonyAlgorithm.getRoll(); }

    int16_t getGxRaw() const { return data.gyro[ImuData::Coordinate::X]; }
    int16_t getGyRaw() const { return data.gyro[ImuData::Coordinate::Y];}
    int16_t getGzRaw() const { return data.gyro[ImuData::Coordinate::Z];}

    int16_t getAxRaw() const { return data.acc[ImuData::Coordinate::X]; }
    int16_t getAyRaw() const { return data.acc[ImuData::Coordinate::Y]; }
    int16_t getAzRaw() const { return data.acc[ImuData::Coordinate::Z]; }

    float getGx() const { return data.gyro[ImuData::Coordinate::X] * BMI088_GYRO_2000_SEN;  }
    float getGy() const { return data.gyro[ImuData::Coordinate::Y] * BMI088_GYRO_2000_SEN;  }
    float getGz() const { return data.gyro[ImuData::Coordinate::Z] * BMI088_GYRO_2000_SEN;  }

    float getAx() const { return data.acc[ImuData::Coordinate::X] * BMI088_ACCEL_3G_SEN; }
    float getAy() const { return data.acc[ImuData::Coordinate::Y] * BMI088_ACCEL_3G_SEN; }
    float getAz() const { return data.acc[ImuData::Coordinate::Z] * BMI088_ACCEL_3G_SEN; }

private:
    struct ImuData
    {
        enum Coordinate
        {
            X=0,
            Y=1,
            Z=2,
        };

        float acc[3] ={};
        float gyro[3] = {};
        float accOffset[3] = {};
        float gyroOffset[3] = {};

        float temperature;
    } data;

    tap::Drivers *drivers;

    ImuState imuState = ImuState::IMU_NOT_CONNECTED;

    Mahony mahonyAlgorithm;

    sensors::ImuHeater imuHeater;

    int calibrationSample = 0;

    void initializeAcc();
    void initializeGyro();

    void computeOffsets();
};

}  // namespace tap::sensors::bmi088

#endif  // BMI088_HPP_
