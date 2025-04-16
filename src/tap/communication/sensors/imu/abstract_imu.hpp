/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_ABSTRACT_IMU_HPP_
#define TAPROOT_ABSTRACT_IMU_HPP_

#include "tap/algorithms/MahonyAHRS.h"
#include "tap/algorithms/transforms/orientation.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/vector.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"

namespace tap::communication::sensors::imu
{
using tap::algorithms::transforms::Orientation;
using tap::algorithms::transforms::Transform;

constexpr float GRAVITY_MPS2 = 9.81f;
class AbstractIMU : public ImuInterface
{
public:
    AbstractIMU(const Transform& mountingTransform = Transform::identity())
        : mountingTransform(mountingTransform)
    {
    }

    void setMountingTransform(const Transform& transform);

    virtual ~AbstractIMU() = default;

    virtual void initialize(float sampleFrequency, float mahonyKp, float mahonyKi);

    /**
     * When this function is called, the bmi088 enters a calibration state during which time,
     * gyro/accel calibration offsets will be computed and the mahony algorithm reset. When
     * calibrating, angle, accelerometer, and gyroscope values will return 0. When calibrating
     * the BMI088 should be level, otherwise the IMU will be calibrated incorrectly.
     */
    virtual void requestCalibration();

    /**
     * Call this function at same rate as intialized sample frequency.
     * Performs the mahony AHRS algorithm to compute pitch/roll/yaw.
     */
    virtual void periodicIMUUpdate();

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
    virtual ImuState getImuState() const { return imuState; }

    inline float getAx() const override { return imuData.accG.x(); }
    inline float getAy() const override { return imuData.accG.y(); }
    inline float getAz() const override { return imuData.accG.z(); }
    inline float getAzMinusG() const { return imuData.accG.z() - GRAVITY_MPS2; }

    inline float getGx() const override { return imuData.gyroDegPerSec.x(); }
    inline float getGy() const override { return imuData.gyroDegPerSec.y(); }
    inline float getGz() const override { return imuData.gyroDegPerSec.z(); }

    inline float getTemp() const override { return imuData.temperature; }

    virtual float getYaw() const override { return mahonyAlgorithm.getYaw(); }
    virtual float getPitch() const override { return mahonyAlgorithm.getPitch(); }
    virtual float getRoll() const override { return mahonyAlgorithm.getRoll(); }

    struct ImuData
    {
        tap::algorithms::transforms::Vector accRaw = {0, 0, 0};
        tap::algorithms::transforms::Vector gyroRaw = {0, 0, 0};
        tap::algorithms::transforms::Vector accOffsetRaw = {0, 0, 0};
        tap::algorithms::transforms::Vector gyroOffsetRaw = {0, 0, 0};
        tap::algorithms::transforms::Vector accG = {0, 0, 0};
        tap::algorithms::transforms::Vector gyroDegPerSec = {0, 0, 0};

        float temperature = 0;
    };

    void setCalibrationSamples(int sampleCount) { offsetSampleCount = sampleCount; }

protected:
    void resetOffsets();
    void computeOffsets();
    void setAccelOffset(float x, float y, float z);
    void setGyroOffset(float x, float y, float z);

    inline void applyTransform(ImuData& data)
    {
        data.accG = mountingTransform.apply(data.accG);
        data.gyroDegPerSec = mountingTransform.apply(data.gyroDegPerSec);
    }

    virtual inline float getAccelerationSensitivity() = 0;

    tap::algorithms::transforms::Transform mountingTransform;

    Mahony mahonyAlgorithm;

    ImuState imuState = ImuState::IMU_NOT_CONNECTED;
    int calibrationSample = 0;
    int offsetSampleCount = 1000;

    ImuData imuData;

    tap::arch::PeriodicMicroTimer readTimeout;

    uint32_t prevIMUDataReceivedTime = 0;
};

}  // namespace tap::communication::sensors::imu

#endif  // TAPROOT_ABSTRACT_IMU_HPP_
