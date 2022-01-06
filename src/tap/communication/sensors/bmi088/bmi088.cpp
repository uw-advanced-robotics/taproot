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

#include "bmi088.hpp"

#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/board/board.hpp"
#include "modm/math/geometry/angle.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/math/units.hpp"

#include "bmi088_data.hpp"
#include "bmi088_hal.hpp"

using namespace modm::literals;
using namespace Board;

namespace tap::sensors::bmi088
{
Bmi088::Bmi088(tap::Drivers *drivers) : drivers(drivers), imuHeater(drivers) {}

Bmi088::ImuState Bmi088::getImuState() const { return imuState; }

void Bmi088::requestRecalibration()
{
    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED)
    {
        data.gyroOffset[ImuData::Coordinate::X] = 0;
        data.gyroOffset[ImuData::Coordinate::Y] = 0;
        data.gyroOffset[ImuData::Coordinate::Z] = 0;
        data.accOffset[ImuData::Coordinate::X] = 0;
        data.accOffset[ImuData::Coordinate::Y] = 0;
        data.accOffset[ImuData::Coordinate::Z] = 0;
        calibrationSample = 0;
        imuState = ImuState::IMU_CALIBRATING;
    }
}

void Bmi088::initiailze()
{
    ImuCS1Accel::GpioOutput();
    ImuCS1Gyro::GpioOutput();

    ImuSpiMaster::connect<ImuMiso::Miso, ImuMosi::Mosi, ImuSck::Sck>();
    ImuSpiMaster::initialize<SystemClock, 10_MHz>();

    modm::delay_ms(1);

    imuState = ImuState::IMU_NOT_CALIBRATED;

    initializeAcc();
    initializeGyro();

    imuHeater.initialize();
}

void Bmi088::initializeAcc()
{
    // Perform system restart
    Bmi088Hal::bmi088AccWriteSingleReg(Acc::ACC_SOFTRESET, Acc::AccSoftreset::ACC_SOFTRESET_VAL);

    // Page 13 of the bmi088 datasheet states:
    // After the POR (power-on reset) the gyroscope is in normal mode, while the accelerometer is in
    // suspend mode. To switch the accelerometer into normal mode, the user must perform the
    // following steps:
    // a. Power up the sensor
    // b. Wait 1 ms
    // c. Enter normal mode by writing '4' to ACC_PWR_CTRL
    // d. Wait for 450 microseconds

    // wait after reset
    modm::delay_ms(1);

    Bmi088Hal::bmi088AccWriteSingleReg(Acc::ACC_PWR_CTRL, Acc::AccPwrCtrl::ACCELEROMETER_ON);

    modm::delay_us(450);

    // read ACC_CHIP_ID to start SPI communication
    // Page 45 of the bmi088 datasheet states:
    // "To change the sensor to SPI mode in the initialization phase, the user
    // could perform a dummy SPI read operation"
    Bmi088Hal::bmi088AccReadSingleReg(Acc::ACC_CHIP_ID);

    // check communication is normal after reset
    uint8_t readChipID = Bmi088Hal::bmi088AccReadSingleReg(Acc::ACC_CHIP_ID);
    if (readChipID != Acc::ACC_CHIP_ID_VALUE)
    {
        RAISE_ERROR(drivers, "bmi088 accel init failed");
        imuState = ImuState::IMU_NOT_CONNECTED;
        return;
    }

    // set acc sensor config and check
    struct AccTuple
    {
        Acc::Register reg;
        Acc::Registers_t value;
    };

    AccTuple bmiAccRegData[] = {
        {Acc::ACC_CONF,
         Acc::AccBandwidth_t(Acc::AccBandwidth::NORMAL) |
             Acc::AccOutputRate_t(Acc::AccOutputRate::Hz800)},
        {Acc::ACC_RANGE, Acc::AccRangeCtrl_t(Acc::AccRangeCtrl::G3)},
    };

    modm::delay_ms(1);

    for (size_t i = 0; i < MODM_ARRAY_SIZE(bmiAccRegData); i++)
    {
        Bmi088Hal::bmi088AccWriteSingleReg(bmiAccRegData[i].reg, bmiAccRegData[i].value);
        modm::delay_us(150);

        uint8_t val = Bmi088Hal::bmi088AccReadSingleReg(bmiAccRegData[i].reg);
        modm::delay_us(150);

        if (val != bmiAccRegData[i].value.value)
        {
            RAISE_ERROR(drivers, "bmi088 acc config failed");
            imuState = ImuState::IMU_NOT_CONNECTED;
            return;
        }
    }
}

void Bmi088::initializeGyro()
{
    // reset gyro
    Bmi088Hal::bmi088GyroWriteSingleReg(Gyro::GYRO_SOFTRESET, Gyro::GyroSoftreset::RESET_SENSOR);
    modm::delay_ms(80);

    // check communication normal after reset
    Bmi088Hal::bmi088GyroReadSingleReg(Gyro::GYRO_CHIP_ID);
    modm::delay_ms(1);
    uint8_t res = Bmi088Hal::bmi088GyroReadSingleReg(Gyro::GYRO_CHIP_ID);
    modm::delay_ms(1);

    if (res != Gyro::GYRO_CHIP_ID_VALUE)
    {
        RAISE_ERROR(drivers, "bmi088 gyro init failed");
        imuState = ImuState::IMU_NOT_CONNECTED;
    }

    struct GyroTuple
    {
        Gyro::Register reg;
        Gyro::Registers_t value;
    };

    // b'1000 0010
    GyroTuple bmiGyroRegData[] = {
        {Gyro::GYRO_RANGE, Gyro::GyroRange::DPS2000},
        {Gyro::GYRO_BANDWIDTH,
         Gyro::GyroBandwidth::ODR1000_BANDWIDTH116 | Gyro::GyroBandwidth_t(0x80)},
        {Gyro::GYRO_LPM1, Gyro::GyroLpm1::PWRMODE_NORMAL},
    };

    // GyroTuple bmiGyroRegData[] = {
    //     {Gyro::GYRO_RANGE, Gyro::GyroRange::DPS2000},
    //     {Gyro::GYRO_BANDWIDTH, Gyro::GyroBandwidth::ODR1000_BANDWIDTH116},
    //     {Gyro::GYRO_LPM1, Gyro::GyroLpm1::PWRMODE_NORMAL},
    //     {Gyro::GYRO_INT_CTRL, Gyro::EnableNewDataInt_t(Gyro::EnableNewDataInt::ENABLED)},
    //     {Gyro::INT3_INT4_IO_CONF,
    //      Gyro::Int3Od_t(Gyro::Int3Od::PUSH_PULL) | Gyro::Int3Lvl_t(Gyro::Int3Lvl::ACTIVE_LOW)},
    //     {Gyro::INT3_INT4_IO_MAP, Gyro::Int3Int4IoMap::DATA_READY_INT3}};

    for (size_t i = 0; i < MODM_ARRAY_SIZE(bmiGyroRegData); i++)
    {
        Bmi088Hal::bmi088GyroWriteSingleReg(bmiGyroRegData[i].reg, bmiGyroRegData[i].value);
        modm::delay_us(150);

        uint8_t val = Bmi088Hal::bmi088GyroReadSingleReg(bmiGyroRegData[i].reg);
        modm::delay_us(150);

        if (val != bmiGyroRegData[i].value.value)
        {
            RAISE_ERROR(drivers, "bmi088 gyro config failed");
            imuState = ImuState::IMU_NOT_CONNECTED;
            return;
        }
    }
}

bool Bmi088::run() { return false; }

#define BIG_ENDIAN_INT16_TO_FLOAT(buff) \
    (static_cast<float>(static_cast<int16_t>((*(buff)) | (*(buff + 1) << 8))))

static inline int16_t parseTemp(uint8_t tempMsb, uint8_t tempLsb)
{
    uint16_t temp = (static_cast<uint16_t>(tempMsb) * 8) + (static_cast<uint16_t>(tempLsb) / 32);

    if (temp > 1023)
    {
        return static_cast<int16_t>(temp) - 2048;
    }
    else
    {
        return static_cast<int16_t>(temp);
    }
}

uint32_t dt = 0;

void Bmi088::periodicIMUUpdate()
{
    if (imuState == ImuState::IMU_NOT_CONNECTED)
    {
        return;
    }

    uint32_t currtime = tap::arch::clock::getTimeMicroseconds();
    uint8_t rxBuff[6] = {};

    Bmi088Hal::bmi088AccReadMultiReg(Acc::ACC_X_LSB, rxBuff, 6);
    data.acc[ImuData::Coordinate::X] = BIG_ENDIAN_INT16_TO_FLOAT(rxBuff);
    data.acc[ImuData::Coordinate::Y] = BIG_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    data.acc[ImuData::Coordinate::Z] = BIG_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    Bmi088Hal::bmi088GyroReadMultiReg(Gyro::RATE_X_LSB, rxBuff, 6);
    data.gyro[ImuData::Coordinate::X] = BIG_ENDIAN_INT16_TO_FLOAT(rxBuff);
    data.gyro[ImuData::Coordinate::Y] = BIG_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    data.gyro[ImuData::Coordinate::Z] = BIG_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    Bmi088Hal::bmi088AccReadMultiReg(Acc::TEMP_MSB, rxBuff, 2);
    data.temperature = static_cast<float>(parseTemp(rxBuff[0], rxBuff[1])) * BMI088_TEMP_FACTOR +
                       BMI088_TEMP_OFFSET;

    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED)
    {
        // TODO fix
        mahonyAlgorithm.updateIMU(modm::toDegree(getGx()), modm::toDegree(getGy()), modm::toDegree(getGz()), getAx(), getAy(), getAz());
    }
    else if (imuState == ImuState::IMU_CALIBRATING)
    {
        computeOffsets();
    }
    else
    {
        RAISE_ERROR(drivers, "imu not connected");
    }

    imuHeater.runTemperatureController(data.temperature);

    dt = tap::arch::clock::getTimeMicroseconds() - currtime;
}

void Bmi088::computeOffsets()
{
    calibrationSample++;

    data.gyroOffset[ImuData::Coordinate::X] += data.gyro[ImuData::Coordinate::X];
    data.gyroOffset[ImuData::Coordinate::Y] += data.gyro[ImuData::Coordinate::Y];
    data.gyroOffset[ImuData::Coordinate::Z] += data.gyro[ImuData::Coordinate::Z];
    data.accOffset[ImuData::Coordinate::X] += data.acc[ImuData::Coordinate::X];
    data.accOffset[ImuData::Coordinate::Y] += data.acc[ImuData::Coordinate::Y];
    data.accOffset[ImuData::Coordinate::Z] +=
        data.acc[ImuData::Coordinate::Z] - ACCELERATION_SENSITIVITY;

    if (calibrationSample >= BMI088_OFFSET_SAMPLES)
    {
        calibrationSample = 0;
        data.gyroOffset[ImuData::Coordinate::X] /= BMI088_OFFSET_SAMPLES;
        data.gyroOffset[ImuData::Coordinate::Y] /= BMI088_OFFSET_SAMPLES;
        data.gyroOffset[ImuData::Coordinate::Z] /= BMI088_OFFSET_SAMPLES;
        data.accOffset[ImuData::Coordinate::X] /= BMI088_OFFSET_SAMPLES;
        data.accOffset[ImuData::Coordinate::Y] /= BMI088_OFFSET_SAMPLES;
        data.accOffset[ImuData::Coordinate::Z] /= BMI088_OFFSET_SAMPLES;
        imuState = ImuState::IMU_CALIBRATED;
        mahonyAlgorithm = Mahony();
    }
}

}  // namespace tap::sensors::bmi088
