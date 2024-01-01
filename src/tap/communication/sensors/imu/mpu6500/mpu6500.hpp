/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_MPU6500_HPP_
#define TAPROOT_MPU6500_HPP_

#include <cstdint>

#include "tap/algorithms/MahonyAHRS.h"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/communication/sensors/imu_heater/imu_heater.hpp"
#include "tap/util_macros.hpp"

#include "modm/math/geometry.hpp"
#include "modm/processing/protothread.hpp"

#define LITTLE_ENDIAN_INT16_TO_FLOAT(buff) \
    (static_cast<float>(static_cast<int16_t>((*(buff) << 8) | *(buff + 1))))

namespace tap
{
class Drivers;
}

namespace tap::communication::sensors::imu::mpu6500
{
/**
 * A class specifically designed for interfacing with the RoboMaster type A board Mpu6500
 * and attached IST8310, connected as an slave device over I2C.
 *
 * To use this class, call Mpu6500::init() to properly initialize and calibrate
 * the MPU6500. Next, call Mpu6500::read() to read acceleration, gyro, and temperature
 * values from the imu. Use the getter methods to access imu information.
 *
 * @note if you are shaking the imu while it is initializing, the offsets will likely
 *      be calibrated poorly and unexpectedly bad results may occur.
 */
class Mpu6500 final_mockable : public ::modm::pt::Protothread, public ImuInterface
{
public:
    /**
     * The number of bytes read to read acceleration, gyro, and temperature.
     * Read 6 bytes for magnetometer data.
     */
    static constexpr uint8_t ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE =
        20;  // From accel data (0x3B) to external sensor data (0x4E)

    /**
     * Storage for the raw data we receive from the mpu6500, as well as offsets
     * that are used each time we receive data.
     */
    struct RawData
    {
        /**
         * Raw acceleration data.
         */
        modm::Vector3f accel;
        /**
         * Raw gyroscope data.
         */
        modm::Vector3f gyro;
        /**
         * Raw temperature.
         */
        uint16_t temperature = 0;
        /**
         * Raw magnetometer data.
         */
        modm::Vector3f magnetometer;
        /**
         * Magnetometer axis offset and scalar.
         */
        modm::Vector3f magnetometerOffset;
        /**
         * Acceleration offset calculated in init.
         */
        modm::Vector3f accelOffset;
        /**
         * Gyroscope offset calculated in init.
         */
        modm::Vector3f gyroOffset;
    };

    /**
     * The number of bytes read to read magnetometer data.
     */
    static constexpr uint8_t MAG_BUFF_RX_SIZE = 6;

    using ProcessRawMpu6500DataFn = void (*)(
        const uint8_t (&)[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE],
        modm::Vector3f &accel,
        modm::Vector3f &gyro,
        modm::Vector3f &mag);

    Mpu6500(Drivers *drivers);
    DISALLOW_COPY_AND_ASSIGN(Mpu6500)
    mockable ~Mpu6500() = default;

    /**
     * Initialize the imu and the SPI line. Uses SPI1, which is internal to the
     * type A board.
     *
     * @note this function can block for approximately 12 seconds.
     */
    mockable void init(float sampleFrequency, float mahonyKp, float mahonyKi);

    /**
     * Calculates the IMU's pitch, roll, and yaw angles usign the Mahony AHRS algorithm.
     * Also runs a controller to keep the temperature constant.
     * Call at 500 hz for best performance.
     */
    mockable void periodicIMUUpdate();

    /**
     * Read data from the imu. This is a protothread that reads the SPI bus using
     * nonblocking I/O.
     *
     * @return `true` if the function is not done, `false` otherwise
     */
    mockable bool read();

    /**
     * Returns the state of the IMU. Can be not connected, connected but not calibrated, calibrating
     * or calibrated. When not connected, IMU data is undefined. When not calibrated, IMU data is
     * valid but the computed yaw angle data will drift. When calibrating, the IMU data is invalid.
     * When calibrated, the IMU data is valid and assuming proper calibration the IMU data should
     * not drift.
     *
     * To be safe, whenever you call functions that return IMU (acceleration, gyroscope,
     * temperature, and angle) data, call this function to ensure the data you are about to receive
     * is not undefined.
     */
    mockable inline ImuState getImuState() const { return imuState; }

    virtual inline const char *getName() const { return "mpu6500"; }

    /**
     * If the imu is not initialized, logs an error and returns 0.
     * Otherwise, returns the value passed in.
     */
    inline float validateReading(float reading)
    {
        if (imuState == ImuState::IMU_CALIBRATED)
        {
            return reading;
        }
        else if (imuState == ImuState::IMU_NOT_CALIBRATED)
        {
            errorState |= 1 << static_cast<uint8_t>(ImuState::IMU_NOT_CALIBRATED);
            return reading;
        }
        else if (imuState == ImuState::IMU_CALIBRATING || imuState == ImuState::IMU_CALIBRATING_MAGNETOMETER)
        {
            errorState |= 1 << static_cast<uint8_t>(ImuState::IMU_CALIBRATING);
            return 0.0f;
        }
        else
        {
            errorState |= 1 << static_cast<uint8_t>(ImuState::IMU_NOT_CONNECTED);
            return 0.0f;
        }
    }

    /**
     * Returns the acceleration reading in the x direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    inline float getAx() final_mockable
    {
        return validateReading(
            static_cast<float>(raw.accel.x - raw.accelOffset.x) * ACCELERATION_GRAVITY /
            ACCELERATION_SENSITIVITY);
    }

    /**
     * Returns the acceleration reading in the y direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    inline float getAy() final_mockable
    {
        return validateReading(
            static_cast<float>(raw.accel.y - raw.accelOffset.y) * ACCELERATION_GRAVITY /
            ACCELERATION_SENSITIVITY);
    }

    /**
     * Returns the acceleration reading in the z direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    inline float getAz() final_mockable
    {
        return validateReading(
            static_cast<float>(raw.accel.z - raw.accelOffset.z) * ACCELERATION_GRAVITY /
            ACCELERATION_SENSITIVITY);
    }

    /**
     * Returns the gyroscope reading in the x direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    inline float getGx() final_mockable
    {
        return validateReading(
            static_cast<float>(raw.gyro.x - raw.gyroOffset.x) / LSB_D_PER_S_TO_D_PER_S);
    }

    /**
     * Returns the gyroscope reading in the y direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    inline float getGy() final_mockable
    {
        return validateReading(
            static_cast<float>(raw.gyro.y - raw.gyroOffset.y) / LSB_D_PER_S_TO_D_PER_S);
    }

    /**
     * Returns the gyroscope reading in the z direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    inline float getGz() final_mockable
    {
        return validateReading(
            static_cast<float>(raw.gyro.z - raw.gyroOffset.z) / LSB_D_PER_S_TO_D_PER_S);
    }

    /**
     * Returns the temperature of the imu in degrees C.
     *
     * @see page 33 of this datasheet:
     * https://3cfeqx1hf82y3xcoull08ihx-wpengine.netdna-ssl.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf
     * for what the magic numbers are used.
     */
    inline float getTemp() final_mockable
    {
        return 21.0f + static_cast<float>(raw.temperature) / 333.87f;
    }

    /**
     * Returns yaw angle. in degrees.
     */
    inline float getYaw() final_mockable { 
        return validateReading(mahonyAlgorithm.getYaw()); }

    /**
     * Returns pitch angle in degrees.
     */
    inline float getPitch() final_mockable { return validateReading(mahonyAlgorithm.getPitch()); }

    /**
     * Returns roll angle in degrees.
     */
    inline float getRoll() final_mockable { return validateReading(mahonyAlgorithm.getRoll()); }

    /**
     * Returns the magnetometer head in the xy plane.
     */
    inline float getMagneticHeading() mockable
    {
        return validateReading(modm::toDegree(atan2f(raw.magnetometer.y, raw.magnetometer.x)));
    }

    mockable inline uint32_t getPrevIMUDataReceivedTime() const { return prevIMUDataReceivedTime; }

    /**
     * Returns the angle difference between the normal vector of the plane that the
     * type A board lies on and of the angle directly upward.
     */
    mockable float getTiltAngle();

    /**
     * Uninitializes the mpu6500 and enters calibration mode.
     */
    mockable void requestCalibration();

    void attachProcessRawMpu6500DataFn(ProcessRawMpu6500DataFn fn) { processRawMpu6500DataFn = fn; }

    /**
     * Use for converting from gyro values we receive to more conventional degrees / second.
     */
    static constexpr float LSB_D_PER_S_TO_D_PER_S = 16.384f;

private:
    static constexpr float ACCELERATION_GRAVITY = 9.80665f;

    /**
     * Use to convert the raw acceleration into more conventional degrees / second^2
     */
    static constexpr float ACCELERATION_SENSITIVITY = 4096.0f;

    /**
     * The number of samples we take while calibrating in order to determine the mpu offsets.
     */
    float MPU6500_OFFSET_SAMPLES = 4000;

    /**
     * The time to read the registers in nonblocking mode, in microseconds.
     */
    static constexpr int NONBLOCKING_TIME_TO_READ_REG = 450;

    /**
     * Time in ms to wait for the IMU heat to stabalize upon initialization.
     */
    static constexpr uint32_t MAX_WAIT_FOR_IMU_TEMPERATURE_STABALIZE = 10'000;

    /**
     * Time in ms to wait after IMU heat has reached stable point upon initialization.
     */
    static constexpr uint32_t WAIT_TIME_AFTER_CALIBRATION = 10'000;

    /**
     * Bit appended or removed from a register while reading/writing.
     */
    static constexpr uint8_t MPU6500_READ_BIT = 0x80;

    Drivers *drivers;

    ProcessRawMpu6500DataFn processRawMpu6500DataFn;

    int delayBtwnCalcAndReadReg = 2000 - NONBLOCKING_TIME_TO_READ_REG;

    ImuState imuState = ImuState::IMU_NOT_CONNECTED;

    tap::arch::MicroTimeout readRegistersTimeout;
    uint8_t tx = 0;  ///< Byte used for reading data in the read protothread
    uint8_t rx = 0;  ///< Byte used for reading data in the read protothread

    RawData raw;

    Mahony mahonyAlgorithm;

    imu_heater::ImuHeater imuHeater;

    float tiltAngle = 0.0f;
    bool tiltAngleCalculated = false;

    uint8_t txBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};

    uint8_t rxBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};

    int calibrationSample = 0;

    uint8_t errorState = 0;

    uint32_t prevIMUDataReceivedTime = 0;

    modm::Vector3f calibrationMaxReading;
    modm::Vector3f calibrationMinReading;

    modm::Vector3f normalizedMagnetometer;

    /**
     * The number of samples we take while calibrating in order to determine the mpu offsets.
     */
    static constexpr float MPU6500_MAGNETOMETER_CALIBRATION_SAMPLES = 2500;

    // Functions for interacting with hardware directly.

    /**
     * Pull the NSS pin low to initiate contact with the imu.
     */
    void mpuNssLow();

    /**
     * Pull the NSS pin high to end contact with the imu.
     */
    void mpuNssHigh();

    /**
     * Write to a given register.
     */
    void spiWriteRegister(uint8_t reg, uint8_t data);

    /**
     * Read from a given register.
     */
    uint8_t spiReadRegister(uint8_t reg);

    /**
     * Read from several registers.
     * regAddr is the first address read, and it reads len number of addresses
     * from that point.
     */
    void spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len);

    /**
     * Add any errors to the error handler that have came up due to calls to validateReading.
     */
    void addValidationErrors();

    /// Default processing function when IMU is lying flat on the robot.
    static void defaultProcessRawMpu6500Data(
        const uint8_t (&rxBuff)[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE],
        modm::Vector3f &accel,
        modm::Vector3f &gyro,
        modm::Vector3f &mag);

    void ist8310Init();

    void writeIST8310Register(uint8_t reg, uint8_t data);

    inline void normalizeMagnetometerReading()
    {

        normalizedMagnetometer = raw.magnetometer - raw.magnetometerOffset;

        if (raw.magnetometerOffset.x != 0)
        {
            normalizedMagnetometer.x /= raw.magnetometerOffset.x;
        } else {
            normalizedMagnetometer.x /= calibrationMaxReading.x - raw.magnetometerOffset.x;
        }

        if (raw.magnetometerOffset.y != 0)
        {
            normalizedMagnetometer.y /= raw.magnetometerOffset.y;
        } else {
            normalizedMagnetometer.y /= calibrationMaxReading.y - raw.magnetometerOffset.y;
        }

        if (raw.magnetometerOffset.z != 0)
        {
            normalizedMagnetometer.z /= raw.magnetometerOffset.z;
        } else {
            normalizedMagnetometer.z /= calibrationMaxReading.z - raw.magnetometerOffset.z;
        }
    }

    bool requestCalibrationFlagDebug = false;
};

}  // namespace tap::communication::sensors::imu::mpu6500

#endif  // TAPROOT_MPU6500_HPP_
