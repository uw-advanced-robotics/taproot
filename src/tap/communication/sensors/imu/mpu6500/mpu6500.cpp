/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "mpu6500.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/board/board.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "mpu6500_config.hpp"
#include "mpu6500_reg.hpp"

#include "tap/communication/sensors/imu/ist8310/ist8310_config.hpp"
#include "tap/communication/sensors/imu/ist8310/ist8310_reg.hpp"

using namespace modm::literals;
using namespace tap::arch;

namespace tap::communication::sensors::imu::mpu6500
{
Mpu6500::Mpu6500(Drivers *drivers)
    : drivers(drivers),
      processRawMpu6500DataFn(Mpu6500::defaultProcessRawMpu6500Data),
      raw(),
      imuHeater(drivers)
{
}

void Mpu6500::requestCalibration()
{
    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED)
    {
        raw.gyroOffset.x = 0;
        raw.gyroOffset.y = 0;
        raw.gyroOffset.z = 0;
        raw.accelOffset.x = 0;
        raw.accelOffset.y = 0;
        raw.accelOffset.z = 0;
        calibrationSample = 0;
        imuState = ImuState::IMU_CALIBRATING;
    }
}

void Mpu6500::init(float sampleFrequency, float mahonyKp, float mahonyKi)
{
#ifndef PLATFORM_HOSTED
    // Configure NSS pin
    Board::ImuNss::GpioOutput();

    // connect GPIO pins to the alternate SPI function
    Board::ImuSpiMaster::connect<Board::ImuMiso::Miso, Board::ImuMosi::Mosi, Board::ImuSck::Sck>();

    // initialize SPI with clock speed
    Board::ImuSpiMaster::initialize<Board::SystemClock, 703125_Hz>();

    // See page 42 of the mpu6500 register map for initialization process:
    // https://3cfeqx1hf82y3xcoull08ihx-wpengine.netdna-ssl.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf
    //
    // When using SPI interface, user should use PWR_MGMT_1 (register 107) as well as
    // SIGNAL_PATH_RESET (register 104) to ensure the reset is performed properly. The sequence
    // used should be:
    //  1. Set H_RESET = 1 (register PWR_MGMT_1)
    //  2. Wait 100ms
    //  3. Set GYRO_RST = ACCEL_RST = TEMP_RST = 1 (register SIGNAL_PATH_RESET)
    //  4. Wait 100ms

    // set power mode
    spiWriteRegister(MPU6500_PWR_MGMT_1, MPU6500_PWR_MGMT_1_DEVICE_RESET_BIT);

    modm::delay_ms(100);

    // reset gyro, accel, and temperature
    spiWriteRegister(MPU6500_SIGNAL_PATH_RESET, MPU6500_SIGNAL_PATH_RESET_ALL);

    modm::delay_ms(100);

    // verify mpu register ID
    if (MPU6500_ID != spiReadRegister(MPU6500_WHO_AM_I))
    {
        RAISE_ERROR(drivers, "Failed to initialize the IMU properly");
        return;
    }

    // Configure mpu
    spiWriteRegister(MPU6500_PWR_MGMT_1, MPU6500_PWR_MGMT_1_CLKSEL);
    modm::delay_ms(1);  // Delay for some time to wait for the register to be updated (probably not
                        // necessary but we do it anyway)
    spiWriteRegister(MPU6500_PWR_MGMT_2, 0x00);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_CONFIG, MPU6500_CONFIG_DATA);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_GYRO_CONFIG, MPU6500_GYRO_CONFIG_DATA);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_ACCEL_CONFIG, MPU6500_ACCEL_CONFIG_DATA);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_ACCEL_CONFIG_2, MPU6500_ACCEL_CONFIG_2_DATA);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_USER_CTRL, MPU6500_USER_CTRL_DATA);
    modm::delay_ms(1);

    // Configure IST8310
    ist8310Init();
#endif

    imuHeater.initialize();

    delayBtwnCalcAndReadReg =
        static_cast<int>(1e6f / sampleFrequency) - NONBLOCKING_TIME_TO_READ_REG;

    assert(delayBtwnCalcAndReadReg >= 0);

    readRegistersTimeout.restart(delayBtwnCalcAndReadReg);

    mahonyAlgorithm.begin(sampleFrequency, mahonyKp, mahonyKi);

    imuState = ImuState::IMU_NOT_CALIBRATED;
}

void Mpu6500::periodicIMUUpdate()
{
    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED)
    {
        mahonyAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
        tiltAngleCalculated = false;
        // Start reading registers in DELAY_BTWN_CALC_AND_READ_REG us
    }
    else
    {
        calibrationSample++;

        raw.gyroOffset.x += raw.gyro.x;
        raw.gyroOffset.y += raw.gyro.y;
        raw.gyroOffset.z += raw.gyro.z;
        raw.accelOffset.x += raw.accel.x;
        raw.accelOffset.y += raw.accel.y;
        raw.accelOffset.z += raw.accel.z - ACCELERATION_SENSITIVITY;

        if (calibrationSample >= MPU6500_OFFSET_SAMPLES)
        {
            calibrationSample = 0;
            raw.gyroOffset.x /= MPU6500_OFFSET_SAMPLES;
            raw.gyroOffset.y /= MPU6500_OFFSET_SAMPLES;
            raw.gyroOffset.z /= MPU6500_OFFSET_SAMPLES;
            raw.accelOffset.x /= MPU6500_OFFSET_SAMPLES;
            raw.accelOffset.y /= MPU6500_OFFSET_SAMPLES;
            raw.accelOffset.z /= MPU6500_OFFSET_SAMPLES;
            imuState = ImuState::IMU_CALIBRATED;
            mahonyAlgorithm.reset();
        }
    }

    readRegistersTimeout.restart(delayBtwnCalcAndReadReg);

    imuHeater.runTemperatureController(getTemp());

    addValidationErrors();
}

bool Mpu6500::read()
{
#ifndef PLATFORM_HOSTED
    PT_BEGIN();
    while (true)
    {
        PT_WAIT_UNTIL(readRegistersTimeout.execute());

        mpuNssLow();
        tx = MPU6500_ACCEL_XOUT_H | MPU6500_READ_BIT;
        rx = 0;
        txBuff[0] = tx;
        PT_CALL(Board::ImuSpiMaster::transfer(&tx, &rx, 1));
        PT_CALL(Board::ImuSpiMaster::transfer(txBuff, rxBuff, ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE));
        mpuNssHigh();
        (*processRawMpu6500DataFn)(rxBuff, raw.accel, raw.gyro, raw.magnetometer);

        raw.temperature = rxBuff[6] << 8 | rxBuff[7];

        prevIMUDataReceivedTime = tap::arch::clock::getTimeMicroseconds();
    }
    PT_END();
#else
    return false;
#endif
}

float Mpu6500::getTiltAngle()
{
    if (!tiltAngleCalculated)
    {
        tiltAngle = modm::toDegree(acosf(
            cosf(mahonyAlgorithm.getPitchRadians()) * cosf(mahonyAlgorithm.getRollRadians())));
        tiltAngleCalculated = true;
    }
    return validateReading(tiltAngle);
}

// Hardware interface functions (blocking functions, for initialization only)

void Mpu6500::spiWriteRegister(uint8_t reg, uint8_t data)
{
#ifdef PLATFORM_HOSTED
    UNUSED(reg);
    UNUSED(data);
#else
    mpuNssLow();
    uint8_t tx = reg & ~MPU6500_READ_BIT;
    uint8_t rx = 0;  // Unused
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    tx = data;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    mpuNssHigh();
#endif
}

uint8_t Mpu6500::spiReadRegister(uint8_t reg)
{
#ifdef PLATFORM_HOSTED
    UNUSED(reg);
    return 0;
#else
    mpuNssLow();
    uint8_t tx = reg | MPU6500_READ_BIT;
    uint8_t rx = 0;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    mpuNssHigh();
    return rx;
#endif
}

void Mpu6500::spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len)
{
#ifdef PLATFORM_HOSTED
    UNUSED(regAddr);
    UNUSED(pData);
    UNUSED(len);
#else
    mpuNssLow();
    uint8_t tx = regAddr | MPU6500_READ_BIT;
    uint8_t rx = 0;
    txBuff[0] = tx;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(txBuff, pData, len);
    mpuNssHigh();
#endif
}

void Mpu6500::mpuNssLow()
{
#ifndef PLATFORM_HOSTED
    Board::ImuNss::setOutput(modm::GpioOutput::Low);
#endif
}

void Mpu6500::mpuNssHigh()
{
#ifndef PLATFORM_HOSTED
    Board::ImuNss::setOutput(modm::GpioOutput::High);
#endif
}

void Mpu6500::writeIST8310Register(uint8_t regAddr, uint8_t data)
{
#ifdef PLATFORM_HOSTED
    UNUSED(regAddr);
    UNUSED(data);
#else
    spiWriteRegister(MPU6500_I2C_SLV1_CTRL, 0x00); // Turn off slave 1
    modm::delay_ms(2);
    spiWriteRegister(MPU6500_I2C_SLV1_REG, regAddr);
    modm::delay_ms(2);
    spiWriteRegister(MPU6500_I2C_SLV1_DO, data);
    modm::delay_ms(2);
    spiWriteRegister(MPU6500_I2C_SLV1_CTRL, MPU6500_READ_BIT | 0x01); // Turn on slave 1, 1 byte
    modm::delay_ms(10);
#endif
}

uint8_t Mpu6500::readIST8310Registers(uint8_t regAddr)
{
#ifdef PLATFORM_HOSTED
    UNUSED(regAddr);
    UNUSED(pData);
    UNUSED(len);
#else
    spiWriteRegister(MPU6500_I2C_SLV4_REG, regAddr);
    modm::delay_ms(10);
    spiWriteRegister(MPU6500_I2C_SLV4_CTRL, MPU6500_READ_BIT);
    modm::delay_ms(10);
    uint8_t data = spiReadRegister(MPU6500_I2C_SLV4_DI);
    // Turn off slave 4 after read
    spiWriteRegister(MPU6500_I2C_SLV4_CTRL, 0x00);
    modm::delay_ms(10);
    return data;
#endif
}

// The UIUC way
/**
 *   WriteReg(MPU6500_USER_CTRL, 0x30);     // enable I2C master and reset all slaves
  WriteReg(MPU6500_I2C_MST_CTRL, 0x0d);  // 400 kHz I2C clock
  // slave 0 for auto receive
  WriteReg(MPU6500_I2C_SLV0_ADDR, 0x0e | 0x80);  // read from device 0x0e
  WriteReg(MPU6500_I2C_SLV0_REG, 0x03);          // read data from 0x03 reg
  // slave 1 for auto transmit
  WriteReg(MPU6500_I2C_SLV1_ADDR, 0x0e);  // write into device 0x0e
  WriteReg(MPU6500_I2C_SLV1_REG, 0x0a);   // write data into 0x0a reg
  WriteReg(MPU6500_I2C_SLV1_DO, 0x01);    // send measurement command
  // enable slave 0 and 1
  WriteReg(MPU6500_I2C_SLV0_CTRL, 0xd6);  // swap endian + 6 bytes rx
  WriteReg(MPU6500_I2C_SLV1_CTRL, 0x81);  // 1 bytes tx
*/
void Mpu6500::ist8310Init(){
    spiWriteRegister(MPU6500_USER_CTRL, 0x30); // enable I2C master mode, reset slaves
    modm::delay_ms(10);
    spiWriteRegister(MPU6500_I2C_MST_CTRL, 0x0D); // 400 kHz I2C clock speed
    modm::delay_ms(10);

    // Slave 0 for auto receive
    spiWriteRegister(MPU6500_I2C_SLV0_ADDR, IST8310_IIC_ADDRESS | MPU6500_READ_BIT); // read from device 0x0e
    modm::delay_ms(10);
    spiWriteRegister(MPU6500_I2C_SLV0_REG, IST8310_DATA_START_ADDRESS); // read data from 0x03 reg
    modm::delay_ms(10);
    
    // Slave 1 for auto transmit
    spiWriteRegister(MPU6500_I2C_SLV1_ADDR, IST8310_IIC_ADDRESS); // write into device 0x0e
    modm::delay_ms(10);
    spiWriteRegister(MPU6500_I2C_SLV1_REG, IST8310_CONTROL_REGISTER1); // write data into 0x0a reg
    modm::delay_ms(10);
    spiWriteRegister(MPU6500_I2C_SLV1_DO, IST8310_SINGLE_MEASUREMENT_MODE); // send measurement command
    modm::delay_ms(10);
    // enable slave 0 and 1
    spiWriteRegister(MPU6500_I2C_SLV0_CTRL, I2C_SLAVE_READ_CONFIG | IST8310_DATA_LENGTH); // swap endian + 6 bytes rx
    modm::delay_ms(10);
    spiWriteRegister(MPU6500_I2C_SLV1_CTRL, 0x01 | MPU6500_READ_BIT); // 1 bytes tx
}

// According to robomaster this is what they do
// void Mpu6500::ist8310Init(){
//     spiWriteRegister(MPU6500_USER_CTRL, 0x30); // enable I2C master mode, reset slaves
//     modm::delay_ms(10);
//     spiWriteRegister(MPU6500_I2C_MST_CTRL, 0x0D); // 400 kHz I2C clock speed
//     modm::delay_ms(10);

//     // Turn on Slave 1 to write data
//     spiWriteRegister(MPU6500_I2C_SLV1_ADDR, IST8310_IIC_ADDRESS);
//     modm::delay_ms(10);
//     // Turn on Slave 4 to read data 
//     spiWriteRegister(MPU6500_I2C_SLV4_ADDR, MPU6500_READ_BIT | IST8310_IIC_ADDRESS);
//     modm::delay_ms(10);

//     // Reset IST8310 
//     writeIST8310Register(IST8310_CONTROL_REGISTER2_DATA, IST8310_SOFT_RESET);
//     modm::delay_ms(10);

//     // Check IST8310 ID
//     uint8_t id = readIST8310Registers(IST8310_WHO_AM_I);
//     if(id != IST8310_DEVICE_ID){
//         RAISE_ERROR(drivers, "Device ID mismatch");
//         return;
//     }

//     // Config IST8310
//     writeIST8310Register(IST8310_CONTROL_REGISTER2, IST8310_CONTROL_REGISTER2_DATA);
//     modm::delay_ms(10);

//     writeIST8310Register(IST8310_PULSE_DURATION_CONTROL_REGISTER, IST8310_PULSE_DURATION_CONTROL_REGISTER_DATA);
//     modm::delay_ms(10);

//     writeIST8310Register(IST8310_AVERAGE_CONTROL_REGISTER, IST8310_AVERAGE_CONTROL_REGISTER_DATA);
//     modm::delay_ms(10);

//     // Turn off slave 1 and 4
//     spiWriteRegister(MPU6500_I2C_SLV1_CTRL, 0x00);
//     modm::delay_ms(10);
//     spiWriteRegister(MPU6500_I2C_SLV4_CTRL, 0x00);
//     modm::delay_ms(10);

//     mpuI2CAutoReadSetup();
// }

// void Mpu6500::mpuI2CAutoReadSetup()
// {
//     // Slave 1 writes data
//     spiWriteRegister(MPU6500_I2C_SLV1_ADDR, IST8310_IIC_ADDRESS);
//     modm::delay_ms(2);
//     spiWriteRegister(MPU6500_I2C_SLV1_REG, IST8310_CONTROL_REGISTER1);
//     modm::delay_ms(2);
//     spiWriteRegister(MPU6500_I2C_SLV1_DO, IST8310_SINGLE_MEASUREMENT_MODE);
//     modm::delay_ms(2);

//     // Slave 0 setup to auto read data
//     spiWriteRegister(MPU6500_I2C_SLV0_ADDR, MPU6500_READ_BIT | IST8310_IIC_ADDRESS);
//     modm::delay_ms(2);
//     spiWriteRegister(MPU6500_I2C_SLV0_REG, IST8310_DATA_START_ADDRESS);
//     modm::delay_ms(2);

    
//     spiWriteRegister(MPU6500_I2C_SLV4_CTRL, IST8310_DATA_START_ADDRESS);
//     modm::delay_ms(2);
//     // Enable access delay for slave 0 and 1
//     spiWriteRegister(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
//     modm::delay_ms(2);
//     // Enable slave 1
//     spiWriteRegister(MPU6500_I2C_SLV1_CTRL, MPU6500_READ_BIT | 0x01);
//     modm::delay_ms(IST8310_SLOW_REFRESH_RATE_MS); // needs to set refresh rate wait between read and write

//     // Enable slave 0
//     spiWriteRegister(MPU6500_I2C_SLV0_CTRL, MPU6500_READ_BIT | IST8310_DATA_LENGTH);
// }

/**
 * Add any errors to the error handler that have came up due to calls to validateReading.
 */
void Mpu6500::addValidationErrors()
{
    if (errorState & (1 << static_cast<uint8_t>(ImuState::IMU_NOT_CALIBRATED)))
    {
        RAISE_ERROR(drivers, "IMU data requested but IMU not calibrated");
    }
    else if (errorState & (1 << static_cast<uint8_t>(ImuState::IMU_CALIBRATING)))
    {
        RAISE_ERROR(drivers, "Reading IMU data but IMU calibrating");
    }
    else if (errorState & (1 << static_cast<uint8_t>(ImuState::IMU_NOT_CONNECTED)))
    {
        RAISE_ERROR(drivers, "Failed to initialize IMU properly");
    }

    errorState = 0;
}

void Mpu6500::defaultProcessRawMpu6500Data(
    const uint8_t (&rxBuff)[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE],
    modm::Vector3f &accel,
    modm::Vector3f &gyro,
    modm::Vector3i &mag)
{
    accel.x = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    accel.y = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    accel.z = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    gyro.x = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 8);
    gyro.y = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 10);
    gyro.z = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 12);

    mag.x = rxBuff[14] << 8 | rxBuff[15];
    mag.y = rxBuff[16] << 8 | rxBuff[17];
    mag.z = rxBuff[18] << 8 | rxBuff[19];
}

}  // namespace tap::communication::sensors::imu::mpu6500
