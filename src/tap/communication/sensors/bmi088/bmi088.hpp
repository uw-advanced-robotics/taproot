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
#include "tap/util_macros.hpp"

#include "modm/processing/protothread.hpp"

#include "bmi088_data.hpp"

namespace tap
{
class Drivers;
}

namespace tap::communication::sensors::bmi088
{
/**
 * For register tables and descriptions, refer to the bmi088 datasheet:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 */
class Bmi088 : public Bmi088Data
{
public:
    enum class ImuState
    {
        IMU_NOT_CONNECTED,
        IMU_NOT_CALIBRATED,
        IMU_CALIBRATING,
        IMU_CALIBRATED,
    };

    static constexpr float ACCELERATION_GRAVITY = 9.80665f;

    static constexpr Acc::AccRange_t ACC_RANGE = Acc::AccRange::G3;
    static constexpr Gyro::GyroRange_t GYRO_RANGE = Gyro::GyroRange::DPS2000;
    /**
     * The maximum angular velocity in degrees / second that the gyro can read based on GYRO_RANGE
     * specified above.
     */
    static constexpr float GYRO_RANGE_MAX_DS = 2000.0f;

    static constexpr float BMI088_TEMP_FACTOR = 0.125f;
    static constexpr float BMI088_TEMP_OFFSET = 23.0f;

    /**
     * Used to convert raw gyro values to units of degrees / second. Ratio has units
     * (degrees / second) / gyro counts.
     */
    static constexpr float GYRO_DS_PER_GYRO_COUNT = GYRO_RANGE_MAX_DS / 32767.0f;

    /**
     * Refer to page 27 of the bmi088 datasheet for explination of this equation.
     * Used to convert raw accel values to units m/s^2. Ratio has units (m/s^2) / acc counts.
     */
    static constexpr float ACC_G_PER_ACC_COUNT =
        modm::pow(2, ACC_RANGE.value + 1) * 1.5f * ACCELERATION_GRAVITY / 32768.0f;

    /**
     * The number of samples we take in order to determine the mpu offsets.
     */
    static constexpr float BMI088_OFFSET_SAMPLES = 1000;

    Bmi088(tap::Drivers *drivers);

    /**
     * Starts and configures the bmi088. Blocks for < 200 ms.
     */
    mockable void initiailze();

    /**
     * Call this function at 500 Hz. Reads IMU data and performs the mahony AHRS algorithm to
     * compute pitch/roll/yaw.
     *
     * @note This function blocks for 129 microseconds to read registers from the BMI088.
     */
    mockable void periodicIMUUpdate();

    /**
     * Returns the state of the IMU. Can be not connected, connected but not calibrated, or
     * calibrated. When not connected, IMU data will be garbage. When not calibrated, IMU data is
     * valid but the computed yaw angle data will drift. When calibrating, the IMU data is invalid.
     * When calibrated, the IMU data is valid and assuming proper calibration the IMU data should
     * not drift.
     *
     * To be safe, whenever you call the functions below, call this function to ensure
     * the data you are about to receive is not garbage.
     */
    mockable ImuState getImuState() const;

    /**
     * When this function is called, the bmi088 enters a calibration state during which time,
     * gyro/accel calibration offsets will be computed and the mahony algorithm reset. When
     * calibrating, angle, accelerometer, and gyroscope values will return 0. When calibrating
     * the BMI088 should be level, otherwise the IMU will be calibrated incorrectly.
     */
    mockable void requestRecalibration();

    mockable float getYaw() { return mahonyAlgorithm.getYaw(); }
    mockable float getPitch() { return mahonyAlgorithm.getPitch(); }
    mockable float getRoll() { return mahonyAlgorithm.getRoll(); }

    mockable float getGx() const { return data.gyroDegPerSec[ImuData::X]; }
    mockable float getGy() const { return data.gyroDegPerSec[ImuData::Y]; }
    mockable float getGz() const { return data.gyroDegPerSec[ImuData::Z]; }

    mockable float getAx() const { return data.accG[ImuData::X]; }
    mockable float getAy() const { return data.accG[ImuData::Y]; }
    mockable float getAz() const { return data.accG[ImuData::Z]; }

    mockable float getTemp() const { return data.temperature; }

private:
    struct ImuData
    {
        enum Coordinate
        {
            X = 0,
            Y = 1,
            Z = 2,
        };

        float accRaw[3] = {};
        float gyroRaw[3] = {};
        float accOffsetRaw[3] = {};
        float gyroOffsetRaw[3] = {};
        float accG[3] = {};
        float gyroDegPerSec[3] = {};

        float temperature;
    } data;

    tap::Drivers *drivers;

    ImuState imuState = ImuState::IMU_NOT_CONNECTED;

    Mahony mahonyAlgorithm;

    tap::sensors::ImuHeater imuHeater;

    int calibrationSample = 0;

    void initializeAcc();
    void initializeGyro();

    void computeOffsets();

    void setAndCheckAccRegister(Acc::Register reg, Acc::Registers_t value);

    void setAndCheckGyroRegister(Gyro::Register reg, Gyro::Registers_t value);
};

}  // namespace tap::communication::sensors::bmi088

#endif  // BMI088_HPP_
