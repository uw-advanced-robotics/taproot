#ifndef ENV_SIMULATOR
#include "mpu6500.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"
#include "mpu6500_reg.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/errors/create_errors.hpp"

namespace aruwlib {

namespace sensors {

using namespace modm::literals;

bool Mpu6500::imuInitialized = false;

Mpu6500::RawData Mpu6500::raw;

Mahony Mpu6500::mahonyAlgorithm;

float Mpu6500::tiltAngle;

uint8_t Mpu6500::txBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};

uint8_t Mpu6500::rxBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};

void Mpu6500::init() {
    Board::ImuNss::GpioOutput();

    // connect GPIO pins to the alternate SPI function
    Board::ImuSpiMaster::connect<
        Board::ImuMiso::Miso,
        Board::ImuMosi::Mosi,
        Board::ImuSck::Sck
    >();

    // initialize SPI with clock speed
    Board::ImuSpiMaster::initialize<Board::SystemClock, 703125_Hz>();

    // set power mode
    spiWriteRegister(MPU6500_PWR_MGMT_1, 0x80);

    modm::delayMilliseconds(100);

    // reset gyro, accel, and temp
    spiWriteRegister(MPU6500_SIGNAL_PATH_RESET, 0x07);
    modm::delayMilliseconds(100);

    // verify mpu register ID
    if (MPU6500_ID !=  spiReadRegister(MPU6500_WHO_AM_I)) {
        RAISE_ERROR("failed to initialize the imu properly",
                    aruwlib::errors::Location::MPU6500,
                    aruwlib::errors::ErrorType::IMU_NOT_RECEIVING_PROPERLY);
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
        spiWriteRegister(Mpu6500InitData[i][0], Mpu6500InitData[i][1]);
        modm::delayMilliseconds(1);
    }

    calculateAccOffset();
    calculateGyroOffset();
}

void Mpu6500::read() {
    if (imuInitialized) {
        spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE);
        raw.accel.x = (rxBuff[0] << 8 | rxBuff[1]) - raw.accelOffset.x;
        raw.accel.y = (rxBuff[2] << 8 | rxBuff[3]) - raw.accelOffset.y;
        raw.accel.z = (rxBuff[4] << 8 | rxBuff[5]) - raw.accelOffset.z;

        raw.temp = rxBuff[6] << 8 | rxBuff[7];

        raw.gyro.x = ((rxBuff[8] << 8 | rxBuff[9]) - raw.gyroOffset.x);
        raw.gyro.y = ((rxBuff[10] << 8 | rxBuff[11]) - raw.gyroOffset.y);
        raw.gyro.z = ((rxBuff[12] << 8 | rxBuff[13]) - raw.gyroOffset.z);

        mahonyAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
        tiltAngle = aruwlib::algorithms::radiansToDegrees(acos(
                cos(mahonyAlgorithm.getPitchRadians()) * cos(mahonyAlgorithm.getRollRadians())));
    } else {
        RAISE_ERROR("failed to initialize the imu properly",
                    aruwlib::errors::Location::MPU6500,
                    aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
    }
}

// Getter functions.

bool Mpu6500::initialized() {
    return imuInitialized;
}

float Mpu6500::getAx() {
    return validateReading(static_cast<float>(raw.accel.x) *
                           ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
}

float Mpu6500::getAy() {
    return validateReading(static_cast<float>(raw.accel.y) *
                           ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
}

float Mpu6500::getAz() {
    return validateReading(static_cast<float>(raw.accel.z) *
                           ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
}

float Mpu6500::getGx() {
    return validateReading(static_cast<float>(raw.gyro.x) / LSB_D_PER_S_TO_D_PER_S);
}

float Mpu6500::getGy() {
    return validateReading(static_cast<float>(raw.gyro.y) / LSB_D_PER_S_TO_D_PER_S);
}

float Mpu6500::getGz() {
    return validateReading(static_cast<float>(raw.gyro.z) / LSB_D_PER_S_TO_D_PER_S);
}

float Mpu6500::getTemp() {
    return validateReading(21.0f + static_cast<float>(raw.temp) / 333.87f);
}

float Mpu6500::getYaw() {
    return validateReading(mahonyAlgorithm.getYaw());
}

float Mpu6500::getPitch() {
    return validateReading(mahonyAlgorithm.getPitch());
}

float Mpu6500::getRoll()
{
    return validateReading(mahonyAlgorithm.getRoll());
}

float Mpu6500::getTiltAngle()
{
    return validateReading(tiltAngle);
}

float Mpu6500::validateReading(float reading) {
    if (imuInitialized) {
        return reading;
    }
    RAISE_ERROR("failed to initialize the imu properly",
                aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
    return 0.0f;
}

// Helper functions for calibration.

void Mpu6500::calculateGyroOffset() {
    for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++) {
        spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, 14);
        raw.gyroOffset.x += (rxBuff[8] << 8) | rxBuff[9];
        raw.gyroOffset.y += (rxBuff[10] << 8) | rxBuff[11];
        raw.gyroOffset.z += (rxBuff[12] << 8) | rxBuff[13];
        modm::delayMilliseconds(2);
    }

    raw.gyroOffset.x /= MPU6500_OFFSET_SAMPLES;
    raw.gyroOffset.y /= MPU6500_OFFSET_SAMPLES;
    raw.gyroOffset.z /= MPU6500_OFFSET_SAMPLES;
}

void Mpu6500::calculateAccOffset() {
    for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++) {
        spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, 14);
        raw.accelOffset.x += (rxBuff[0] << 8) | rxBuff[1];
        raw.accelOffset.y += (rxBuff[2] << 8) | rxBuff[3];
        raw.accelOffset.z += ((rxBuff[4] << 8) | rxBuff[5]) - 4096;
        modm::delayMilliseconds(2);
    }

    raw.accelOffset.x /= MPU6500_OFFSET_SAMPLES;
    raw.accelOffset.y /= MPU6500_OFFSET_SAMPLES;
    raw.accelOffset.z /= MPU6500_OFFSET_SAMPLES;
}

// Hardware interface functions.

uint8_t Mpu6500::spiWriteRegister(uint8_t reg, uint8_t data) {
    mpuNssLow();
    uint8_t tx = reg & 0x7F;
    uint8_t rx = 0;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    tx = data;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    mpuNssHigh();
    return 0;
}

uint8_t Mpu6500::spiReadRegister(uint8_t reg) {
    mpuNssLow();
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    mpuNssHigh();
    return rx;
}

uint8_t Mpu6500::spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len) {
    mpuNssLow();
    uint8_t tx = regAddr | 0x80;
    uint8_t rx = 0;
    txBuff[0] = tx;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(txBuff, pData, len);
    mpuNssHigh();
    return 0;
}

void Mpu6500::mpuNssLow() {
    Board::ImuNss::setOutput(modm::GpioOutput::Low);
}

void Mpu6500::mpuNssHigh() {
    Board::ImuNss::setOutput(modm::GpioOutput::High);
}

}  // namespace sensors

}  // namespace aruwlib
#endif
