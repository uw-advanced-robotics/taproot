#include "mpu6500.hpp"
#include "mpu6500_reg.hpp"

namespace aruwlib {

namespace sensors {
    Mpu6500::mpu_info_t Mpu6500::mpu6500Data;

    uint8_t Mpu6500::mpu6500TxBuff[ACC_GYRO_BUFF_RX_SIZE] = {0};

    uint8_t Mpu6500::mpu6500RxBuff[ACC_GYRO_BUFF_RX_SIZE] = {0};

    Mpu6500::mpu_cali_t Mpu6500::imuCaliFlags;

    bool Mpu6500::imuInitialized = false;

    // initialize the imu and SPIbx
    void Mpu6500::init() {
        Board::ImuNcc::GpioOutput();

        // connect GPIO pins to the alternate SPI function
        Board::ImuSpiMaster::connect<
            Board::ImuMiso::Miso,
            Board::ImuMosi::Mosi,
            Board::ImuSck::Sck
        >();

        // initialize SPI with clock speed
        Board::ImuSpiMaster::initialize<Board::SystemClock, 703125_Hz>();

        // set power mode
        mpuWriteReg(MPU6500_PWR_MGMT_1, 0x80);

        modm::delayMilliseconds(100);

        // reset gyro, accel, and temp
        mpuWriteReg(MPU6500_SIGNAL_PATH_RESET, 0x07);
        modm::delayMilliseconds(100);

        // verify mpu register ID
        if (MPU6500_ID !=  mpuReadReg(MPU6500_WHO_AM_I)) {
            // throw NON-FATAL-ERROR-CHECK, imu not receiving properly
            return;
        }

        imuInitialized = true;

        // 0: 250hz; 1: 184hz; 2: 92hz; 3: 41hz; 4: 20hz; 5: 10hz; 6: 5hz; 7: 3600hz
        uint8_t Mpu6500InitData[7][2] = {
            {MPU6500_PWR_MGMT_1, 0x03},      // Auto selects Clock Source
            {MPU6500_PWR_MGMT_2, 0x00},      // all enable
            {MPU6500_CONFIG, 0x02},          // gyro bandwidth 0x00:250Hz 0x04:20Hz
            {MPU6500_GYRO_CONFIG, 0x18},     // gyro range 0x10:+-1000dps 0x18:+-2000dps
            {MPU6500_ACCEL_CONFIG, 0x10},    // acc range 0x10:+-8G
            {MPU6500_ACCEL_CONFIG_2, 0x00},  // acc bandwidth 0x00:250Hz 0x04:20Hz
            {MPU6500_USER_CTRL, 0x20},       // Enable the I2C Master I/F module
                                             // pins ES_DA and ES_SCL are isolated from
                                             // pins SDA/SDI and SCL/SCLK.
        };

        // write init setting to registers
        for (int i = 0; i < 7; i++) {
            mpuWriteReg(Mpu6500InitData[i][0], Mpu6500InitData[i][1]);
            modm::delayMilliseconds(1);
        }

        // offset for gyro drift
        getMpuGyroOffset();

        // offset for accel drift
        getMpuAccOffset();
    }

    // parse imu data from data buffer
    void Mpu6500::read() {
        if (imuInitialized) {
        mpuReadRegs(MPU6500_ACCEL_XOUT_H, mpu6500RxBuff, 14);
            mpu6500Data.ax = (mpu6500RxBuff[0] << 8 | mpu6500RxBuff[1]) - mpu6500Data.ax_offset;
            mpu6500Data.ay = (mpu6500RxBuff[2] << 8 | mpu6500RxBuff[3]) - mpu6500Data.ay_offset;
            mpu6500Data.az = (mpu6500RxBuff[4] << 8 | mpu6500RxBuff[5]) - mpu6500Data.az_offset;
            mpu6500Data.temp = mpu6500RxBuff[6] << 8 | mpu6500RxBuff[7];
            mpu6500Data.gx = ((mpu6500RxBuff[8] << 8 | mpu6500RxBuff[9]) - mpu6500Data.gx_offset);
            mpu6500Data.gy = ((mpu6500RxBuff[10] << 8 | mpu6500RxBuff[11]) - mpu6500Data.gy_offset);
            mpu6500Data.gz = ((mpu6500RxBuff[12] << 8 | mpu6500RxBuff[13]) - mpu6500Data.gz_offset);
        } else {
            // NON-FATAL-ERROR-CHECK
        }
    }

    // get temperature value in C
    float Mpu6500::mpuGetTemp() {
        if (imuInitialized) {
            return 21.0f + static_cast<float>(mpu6500Data.temp) / 333.87f;
        } else {
            // NON-FATAL-ERROR-CHECK
            return NAN;
        }
    }

    // get accleration reading on x-axis
    int16_t Mpu6500::getAx() {
        if (imuInitialized) {
            return mpu6500Data.ax;
        } else {
            // NON-FATAL-ERROR-CHECK
            return -1;
        }
    }

    // get accleration reading on y-axis
    int16_t Mpu6500::getAy() {
        if (imuInitialized) {
            return mpu6500Data.ay;
        } else {
            // NON-FATAL-ERROR-CHECK
            return -1;
        }
    }

    // get acceleration reading on z-axis
    int16_t Mpu6500::getAz() {
        if (imuInitialized) {
            return mpu6500Data.az;
        } else {
            // NON-FATAL-ERROR-CHECK
            return -1;
        }
    }

    // get gyro reading on x-axis
    int16_t Mpu6500::getGx() {
        if (imuInitialized) {
            return mpu6500Data.gx;
        } else {
            // NON-FATAL-ERROR-CHECK
            return -1;
        }
    }

    // get gyro reading on y-axis
    int16_t Mpu6500::getGy() {
        if (imuInitialized) {
            return mpu6500Data.gy;
        } else {
            // NON-FATAL-ERROR-CHECK
            return -1;
        }
    }

    // get gyro reading on z-axis
    int16_t Mpu6500::getGz() {
        if (imuInitialized) {
            return mpu6500Data.gz;
        } else {
            // NON-FATAL-ERROR-CHECK
            return -1;
        }
    }

    // write to a given register
    uint8_t Mpu6500::mpuWriteReg(uint8_t const reg, uint8_t const data) {
        mpuNssLow();
        uint8_t tx = reg & 0x7F;
        uint8_t rx = 0;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        tx = data;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        mpuNssHigh();
        return 0;
    }

    // read from a given register
    uint8_t Mpu6500::mpuReadReg(uint8_t const reg) {
        mpuNssLow();
        uint8_t tx = reg | 0x80;
        uint8_t rx = 0;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        mpuNssHigh();
        return rx;
    }

    // Read from several registers.
    // regAddr is the first address read, and it reads len number of addresses
    // from that point.
    uint8_t Mpu6500::mpuReadRegs(uint8_t const regAddr, uint8_t *pData, uint8_t len) {
        mpuNssLow();
        uint8_t tx = regAddr | 0x80;
        uint8_t rx = 0;
        mpu6500TxBuff[0] = tx;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        Board::ImuSpiMaster::transferBlocking(mpu6500TxBuff, pData, len);
        mpuNssHigh();
        return 0;
    }

    // calibrate gyro offset values
    void Mpu6500::getMpuGyroOffset() {
        for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++) {
            mpuReadRegs(MPU6500_ACCEL_XOUT_H, mpu6500RxBuff, 14);
            mpu6500Data.gx_offset += (mpu6500RxBuff[8] << 8) | mpu6500RxBuff[9];
            mpu6500Data.gy_offset += (mpu6500RxBuff[10] << 8) | mpu6500RxBuff[11];
            mpu6500Data.gz_offset += (mpu6500RxBuff[12] << 8) | mpu6500RxBuff[13];
            modm::delayMilliseconds(2);
        }

        mpu6500Data.gx_offset /= MPU6500_OFFSET_SAMPLES;
        mpu6500Data.gy_offset /= MPU6500_OFFSET_SAMPLES;
        mpu6500Data.gz_offset /= MPU6500_OFFSET_SAMPLES;

        imuCaliFlags.gyroCalcFlag = false;
    }

    // calibrate accelerometer offset values
    // possbie magic number for az: -4096
    void Mpu6500::getMpuAccOffset() {
        for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++) {
            mpuReadRegs(MPU6500_ACCEL_XOUT_H, mpu6500RxBuff, 14);
            mpu6500Data.ax_offset += (mpu6500RxBuff[0] << 8) | mpu6500RxBuff[1];
            mpu6500Data.ay_offset += (mpu6500RxBuff[2] << 8) | mpu6500RxBuff[3];
            mpu6500Data.az_offset += (mpu6500RxBuff[4] << 8) | mpu6500RxBuff[5];
            modm::delayMilliseconds(2);
        }

        mpu6500Data.ax_offset /= MPU6500_OFFSET_SAMPLES;
        mpu6500Data.ay_offset /= MPU6500_OFFSET_SAMPLES;
        mpu6500Data.az_offset /= MPU6500_OFFSET_SAMPLES;

        imuCaliFlags.accCalcFlag = false;
    }

    // calibrates the imu only when we have flags
    void Mpu6500::caliFlagHandler() {
        if (imuCaliFlags.accCalcFlag) {
            getMpuAccOffset();
        }
        if (imuCaliFlags.gyroCalcFlag) {
            getMpuGyroOffset();
        }
    }

    void Mpu6500::mpuNssLow() {
        Board::ImuNcc::setOutput(modm::GpioOutput::Low);
    }

    void Mpu6500::mpuNssHigh() {
        Board::ImuNcc::setOutput(modm::GpioOutput::High);
    }

}  // namespace sensors

}  // namespace aruwlib
