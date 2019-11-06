/*
 * To use this class, call Remote::init() to properly initialize and calibrate
 * the MPU6500. Next, call Remote::read() to read acceleration, gyro, and temp
 * values from the imu. Use the getter methods to access imu information.
 */

#ifndef MPU6500_H
#define MPU6500_H

#include <rm-dev-board-a/board.hpp>

using namespace modm::literals;

namespace aruwlib {

namespace sensors {

class  Mpu6500 {
 public:
    // initialize the imu and SPI
    static void init(void);

    // read data from the imu
    static void read(void);

    static int16_t getAx(void);

    static int16_t getAy(void);

    static int16_t getAz(void);

    static int16_t getGx(void);

    static int16_t getGy(void);

    static int16_t getGz(void);

    // get temperature value in C
    static float mpuGetTemp(void);

    static void caliFlagHandler(void);

 private:
     typedef struct {
        // acceleration data
        int16_t ax = 0;
        int16_t ay = 0;
        int16_t az = 0;

        // gyroscope data
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;

        // temperature
        uint16_t temp = 0;

        // offsets
        int16_t ax_offset = 0;
        int16_t ay_offset = 0;
        int16_t az_offset = 0;

        int16_t gx_offset = 0;
        int16_t gy_offset = 0;
        int16_t gz_offset = 0;
    } mpu_info_t;

    typedef struct {
        bool gyroCalcFlag = true;
        bool accCalcFlag = true;
    } mpu_cali_t;

    #define MPU6500_OFFSET_SAMPLES 1000

    #define ACC_GYRO_BUFF_RX_SIZE 14

    static bool imuInitialized;

    static mpu_info_t mpu6500Data;

    static uint8_t mpu6500TxBuff[ACC_GYRO_BUFF_RX_SIZE];

    static uint8_t mpu6500RxBuff[ACC_GYRO_BUFF_RX_SIZE];

    static mpu_cali_t imuCaliFlags;

    static void mpuNssLow(void);

    static void mpuNssHigh(void);

    static uint8_t mpuWriteReg(uint8_t const reg, uint8_t const data);

    static uint8_t mpuReadReg(uint8_t const reg);

    static uint8_t mpuReadRegs(uint8_t const regAddr, uint8_t *pData, uint8_t len);

    static void getMpuGyroOffset(void);

    static void getMpuAccOffset(void);
};

}  // namespace sensors

}  // namespace aruwlib

#endif
