#ifndef MPU6500_HPP_
#define MPU6500_HPP_

#include <cstdint>

#include "aruwlib/algorithms/MahonyAHRS.h"

namespace aruwlib {

namespace sensors {

/**
 * A class specifically designed for interfacing with the RoboMaster type A board Mpu6500.
 * 
 * To use this class, call Remote::init() to properly initialize and calibrate
 * the MPU6500. Next, call Remote::read() to read acceleration, gyro, and temp
 * values from the imu. Use the getter methods to access imu information.
 * 
 * @note if you are shaking the imu while it is initializing, the offsets will likely
 *      be calibrated poorly and unexpectedly bad results may occur.
 */
class Mpu6500 {
 public:
    Mpu6500() = default;
    Mpu6500(const Mpu6500&) = delete;
    Mpu6500 &operator=(const Mpu6500&) = default;

    /**
     * Initialize the imu and the SPI line. Uses SPI1, which is internal to the
     * type A board.
     * 
     * @note this function blocks for approximately 1 second.
     */
    void init();

    /**
     * Read data from the imu. Call at 500 hz for best performance.
     */ 
    void read();

    /**
     * To be safe, whenever you call the functions below, call this function to insure
     * the data you are about to receive is not garbage.
     */
    bool initialized() const;

    /**
     * Returns the acceleration reading in the x direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    float getAx() const;

    /**
     * Returns the acceleration reading in the y direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    float getAy() const;

    /**
     * Returns the acceleration reading in the z direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    float getAz() const;

    /**
     * Returns the gyroscope reading in the x direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    float getGx() const;

    /**
     * Returns the gyroscope reading in the y direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    float getGy() const;

    /**
     * Returns the gyroscope reading in the z direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    float getGz() const;

    /**
     * Returns the temperature of the imu in degrees C.
     */
    float getTemp() const;

    /**
     * Returns yaw angle. in degrees.
     */
    float getYaw();

    /**
     * Returns pitch angle in degrees.
     */
    float getPitch();

    /**
     * Returns roll angle in degrees.
     */
    float getRoll();

    /**
     * Returns the angle difference between the normal vector of the plane that the
     * type A board lies on and of the angle directly upward.
     */
    float getTiltAngle() const;

 private:
    static constexpr float ACCELERATION_GRAVITY = 9.80665f;

    ///< Use for converting from gyro values we receive to more conventional degrees / second.
    static constexpr float LSB_D_PER_S_TO_D_PER_S = 16.384f;

    ///< Use to convert the raw acceleration into more conventional degrees / second^2
    static constexpr float ACCELERATION_SENSITIVITY = 4096.0f;

    ///< The number of samples we take in order to determine the mpu offsets.
    static constexpr float MPU6500_OFFSET_SAMPLES = 300;

    ///< The number of bytes read to read acceleration, gyro, and temp.
    static const uint8_t ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE = 14;

    /**
     * Storage for the raw data we receive from the mpu6500, as well as offsets
     * that are used each time we receive data.
     */
    struct RawData {
        ///< Raw acceleration data.
        struct Accel {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
        };

        ///< Raw gyroscope data.
        struct Gyro {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
        };

        ///< Acceleration offset calculated in init.
        struct AccelOffset {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
        };

        ///< Gyroscope offset calculated in init.
        struct GyroOffset {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
        };

        Accel accel;
        Gyro gyro;

        ///< Raw temperature.
        uint16_t temp = 0;

        AccelOffset accelOffset;
        GyroOffset gyroOffset;
    };

    bool imuInitialized = false;

#ifndef ENV_SIMULATOR
    RawData raw;

    Mahony mahonyAlgorithm;

    float tiltAngle = 0.0f;

    uint8_t txBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = { 0 };

    uint8_t rxBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = { 0 };

    ///< Compute the gyro offset values. @note this function blocks.
    void calculateGyroOffset();

    ///< Calibrate accelerometer offset values. @note this function blocks.
    void calculateAccOffset();

    // Functions for interacting with hardware directly.

    ///< Pull the NSS pin low to initiate contact with the imu.
    void mpuNssLow();

    ///< Pull the NSS pin high to end contact with the imu.
    void mpuNssHigh();

    /**
     * If the imu is not initializes, logs an error and returns 0,
     * otherwise returns the value passed in.
     */
    inline float validateReading(float reading) const;

    /**
     * Write to a given register.
     */
    uint8_t spiWriteRegister(uint8_t reg, uint8_t data);

    /**
     * Read from a given register.
     */
    uint8_t spiReadRegister(uint8_t reg);

    /**
     * Read from several registers.
     * regAddr is the first address read, and it reads len number of addresses
     * from that point.
     */
    uint8_t spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len);
#endif
};

}  // namespace sensors

}  // namespace aruwlib

#endif  // MPU6500_HPP_
