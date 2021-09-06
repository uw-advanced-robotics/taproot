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

#include "mpu6500.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"
#include "tap/rm-dev-board-a/board.hpp"

#include "mpu6500_config.hpp"
#include "mpu6500_reg.hpp"

namespace tap
{
namespace sensors
{
using namespace modm::literals;

Mpu6500::Mpu6500(Drivers *drivers)
    : drivers(drivers),
      raw(),
      imuTemperatureController(
          TEMPERATURE_PID_P,
          TEMPERATURE_PID_I,
          TEMPERATURE_PID_D,
          TEMPERATURE_PID_MAX_ERR_SUM,
          TEMPERATURE_PID_MAX_OUT)
{
}

void Mpu6500::init()
{
#ifndef PLATFORM_HOSTED
    // Configure NSS pin
    Board::ImuNss::GpioOutput();

    // connect GPIO pins to the alternate SPI function
    Board::ImuSpiMaster::connect<Board::ImuMiso::Miso, Board::ImuMosi::Mosi, Board::ImuSck::Sck>();

    // initialize SPI with clock speed
    Board::ImuSpiMaster::initialize<Board::SystemClock, 703125_Hz>();

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
        RAISE_ERROR(
            drivers,
            "failed to initialize the imu properly",
            tap::errors::Location::MPU6500,
            tap::errors::Mpu6500ErrorType::IMU_NOT_RECEIVING_PROPERLY);
        return;
    }

    uint8_t mpu6500InitData[7][2] = {
        {MPU6500_PWR_MGMT_1, MPU6500_PWR_MGMT_1_CLKSEL},
        {MPU6500_PWR_MGMT_2, 0x00},  // enable all
        {MPU6500_CONFIG, MPU6500_CONFIG_DATA},
        {MPU6500_GYRO_CONFIG, MPU6500_GYRO_CONFIG_DATA},
        {MPU6500_ACCEL_CONFIG, MPU6500_ACCEL_CONFIG_DATA},
        {MPU6500_ACCEL_CONFIG_2, MPU6500_ACCEL_CONFIG_2_DATA},
        {MPU6500_USER_CTRL, MPU6500_USER_CTRL_DATA},
    };

    // write init setting to registers
    for (unsigned int i = 0; i < sizeof(mpu6500InitData) / 2; i++)
    {
        spiWriteRegister(mpu6500InitData[i][0], mpu6500InitData[i][1]);
        modm::delay_ms(1);
    }

    imuInitialized = true;

    // Initialize the heater timer frequency
    drivers->pwm.setTimerFrequency(gpio::Pwm::TIMER_3, HEATER_PWM_FREQUENCY);

    // Wait for the heater to warm the mpu6500 up
    arch::MilliTimeout waitHeatTimeout(MAX_WAIT_FOR_IMU_TEMPERATURE_STABALIZE);
    do
    {
        readTempAndRunController();
        modm::delay_ms(2);
    } while (!waitHeatTimeout.execute() && getTemp() < IMU_DESIRED_TEMPERATURE);

    // Wait for the IMU temperature to stabalize now that we are close to the correct temperature
    waitHeatTimeout.restart(WAIT_TIME_AFTER_CALIBRATION);
    while (!waitHeatTimeout.execute())
    {
        readTempAndRunController();
        modm::delay_ms(2);
    }

    calculateAccOffset();
    calculateGyroOffset();

    readRegistersTimeout.restart(DELAY_BTWN_CALC_AND_READ_REG);
#endif
}

void Mpu6500::calcIMUAngles()
{
    if (imuInitialized)
    {
        mahonyAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
        tiltAngleCalculated = false;
        // Start reading registers in DELAY_BTWN_CALC_AND_READ_REG us
        readRegistersTimeout.restart(DELAY_BTWN_CALC_AND_READ_REG);

        runTemperatureController();
    }
    else
    {
        RAISE_ERROR(
            drivers,
            "failed to initialize the imu properly",
            tap::errors::Location::MPU6500,
            tap::errors::Mpu6500ErrorType::IMU_DATA_NOT_INITIALIZED);
    }
}

#define LITTLE_ENDIAN_INT16_TO_FLOAT(buff) \
    (static_cast<float>(static_cast<int16_t>((*(buff) << 8) | *(buff + 1))))

bool Mpu6500::read()
{
#ifndef PLATFORM_HOSTED
    PT_BEGIN();
    while (true)
    {
        PT_WAIT_UNTIL(readRegistersTimeout.execute());

        mpuNssLow();
        tx = MPU6500_ACCEL_XOUT_H | 0x80;
        rx = 0;
        txBuff[0] = tx;
        PT_CALL(Board::ImuSpiMaster::transfer(&tx, &rx, 1));
        PT_CALL(Board::ImuSpiMaster::transfer(txBuff, rxBuff, ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE));
        mpuNssHigh();

        raw.accel.x = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff) - raw.accelOffset.x;
        raw.accel.y = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2) - raw.accelOffset.y;
        raw.accel.z = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4) - raw.accelOffset.z;

        raw.temperature = rxBuff[6] << 8 | rxBuff[7];

        raw.gyro.x = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 8) - raw.gyroOffset.x;
        raw.gyro.y = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 10) - raw.gyroOffset.y;
        raw.gyro.z = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 12) - raw.gyroOffset.z;
    }
    PT_END();
#else
    return false;
#endif
}

// Getter functions.

bool Mpu6500::initialized() const { return imuInitialized; }

float Mpu6500::getAx() const
{
    return validateReading(
        static_cast<float>(raw.accel.x) * ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
}

float Mpu6500::getAy() const
{
    return validateReading(
        static_cast<float>(raw.accel.y) * ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
}

float Mpu6500::getAz() const
{
    return validateReading(
        static_cast<float>(raw.accel.z) * ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
}

float Mpu6500::getGx() const
{
    return validateReading(static_cast<float>(raw.gyro.x) / LSB_D_PER_S_TO_D_PER_S);
}

float Mpu6500::getGy() const
{
    return validateReading(static_cast<float>(raw.gyro.y) / LSB_D_PER_S_TO_D_PER_S);
}

float Mpu6500::getGz() const
{
    return validateReading(static_cast<float>(raw.gyro.z) / LSB_D_PER_S_TO_D_PER_S);
}

float Mpu6500::getTemp() const
{
    return validateReading(21.0f + static_cast<float>(raw.temperature) / 333.87f);
}

float Mpu6500::getYaw() { return validateReading(mahonyAlgorithm.getYaw()); }

float Mpu6500::getPitch() { return validateReading(mahonyAlgorithm.getPitch()); }

float Mpu6500::getRoll() { return validateReading(mahonyAlgorithm.getRoll()); }

float Mpu6500::getTiltAngle()
{
    if (!tiltAngleCalculated)
    {
        tiltAngle = tap::algorithms::radiansToDegrees(acosf(
            cosf(mahonyAlgorithm.getPitchRadians()) * cosf(mahonyAlgorithm.getRollRadians())));
        tiltAngleCalculated = true;
    }
    return validateReading(tiltAngle);
}

float Mpu6500::validateReading(float reading) const
{
    if (imuInitialized)
    {
        return reading;
    }
    RAISE_ERROR(
        drivers,
        "failed to initialize the imu properly",
        tap::errors::Location::MPU6500,
        tap::errors::Mpu6500ErrorType::IMU_DATA_NOT_INITIALIZED);
    return 0.0f;
}

// Helper functions for calibration.

void Mpu6500::calculateGyroOffset()
{
#ifndef PLATFORM_HOSTED
    for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++)
    {
        spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, 14);
        raw.gyroOffset.x += static_cast<int16_t>((rxBuff[8] << 8) | rxBuff[9]);
        raw.gyroOffset.y += static_cast<int16_t>((rxBuff[10] << 8) | rxBuff[11]);
        raw.gyroOffset.z += static_cast<int16_t>((rxBuff[12] << 8) | rxBuff[13]);
        raw.temperature = rxBuff[6] << 8 | rxBuff[7];
        runTemperatureController();
        modm::delay_ms(2);
    }

    raw.gyroOffset.x /= MPU6500_OFFSET_SAMPLES;
    raw.gyroOffset.y /= MPU6500_OFFSET_SAMPLES;
    raw.gyroOffset.z /= MPU6500_OFFSET_SAMPLES;
#endif
}

void Mpu6500::calculateAccOffset()
{
#ifndef PLATFORM_HOSTED
    for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++)
    {
        spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, 14);
        raw.accelOffset.x += static_cast<int16_t>((rxBuff[0] << 8) | rxBuff[1]);
        raw.accelOffset.y += static_cast<int16_t>((rxBuff[2] << 8) | rxBuff[3]);
        raw.accelOffset.z += static_cast<int16_t>(((rxBuff[4] << 8) | rxBuff[5]) - 4096);
        raw.temperature = rxBuff[6] << 8 | rxBuff[7];
        runTemperatureController();
        modm::delay_ms(2);
    }

    raw.accelOffset.x /= MPU6500_OFFSET_SAMPLES;
    raw.accelOffset.y /= MPU6500_OFFSET_SAMPLES;
    raw.accelOffset.z /= MPU6500_OFFSET_SAMPLES;
#endif
}

// Hardware interface functions (blocking functions, for initialization only)

void Mpu6500::spiWriteRegister(uint8_t reg, uint8_t data)
{
#ifndef PLATFORM_HOSTED
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
#ifndef PLATFORM_HOSTED
    mpuNssLow();
    uint8_t tx = reg | MPU6500_READ_BIT;
    uint8_t rx = 0;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    mpuNssHigh();
    return rx;
#else
    return 0;
#endif
}

void Mpu6500::spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len)
{
#ifndef PLATFORM_HOSTED
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

void Mpu6500::runTemperatureController()
{
    // Run PID controller to find desired output, output units PWM frequency
    imuTemperatureController.update(IMU_DESIRED_TEMPERATURE - getTemp());

    // Set heater PWM output, limit output so it is not < 0
    drivers->pwm.write(std::max(0.0f, imuTemperatureController.getValue()), tap::gpio::Pwm::HEATER);
}

void Mpu6500::readTempAndRunController()
{
    spiReadRegisters(MPU6500_TEMP_OUT_H, rxBuff, 2);
    raw.temperature = rxBuff[0] << 8 | rxBuff[1];
    runTemperatureController();
}

}  // namespace sensors

}  // namespace tap
