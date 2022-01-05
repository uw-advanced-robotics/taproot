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
#include "bmi088_hal.hpp"

#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/board/board.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/math/units.hpp"

#include "bmi088_data.hpp"

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
    ImuSpiMaster::connect<ImuMiso::Miso, ImuMosi::Mosi, ImuSck::Sck>();
    ImuSpiMaster::initialize<SystemClock, 10_MHz>();

    modm::delay_ms(1);

    initializeAcc();
    initializeGyro();

    imuHeater.initialize();

    imuState = ImuState::IMU_NOT_CALIBRATED;
}

void Bmi088::initializeAcc()
{
    // read ACC_CHIP_ID to start SPI communication
    bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    // acc softreset
    bmi088AccWriteSingleReg(Acc::Register::ACC_SOFTRESET, Acc::AccSoftreset::ACC_SOFTRESET_VAL);
    modm::delay_us(BMI088_COMM_LONG_WAIT_TIME);

    // check communication is normal after reset
    bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    uint8_t res = bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    // check chip ID
    if (res != Acc::ACC_CHIP_ID)
    {
        RAISE_ERROR(drivers, "bmi088 gyro init failed");
        return;
    }

    // set acc sensor config and check
    struct AccTuple
    {
        Acc::Register reg;
        Acc::Registers_t value;
    };

    AccTuple bmiAccRegData[] = {
        {Acc::Register::ACC_PWR_CTRL, Acc::AccPwrCtrl::ACCELEROMETER_ON},
        {Acc::Register::ACC_PWR_CONF, Acc::AccPwrConf::ACTIVE_MODE},
        {Acc::Register::ACC_CONF,
         Acc::AccBandwidth_t(Acc::AccBandwidth::NORMAL) |
             Acc::AccOutputRate_t(Acc::AccOutputRate::Hz800)},  // potentially also | 0x80????
        {Acc::Register::ACC_RANGE, Acc::AccRangeCtrl_t(Acc::AccRangeCtrl::G3)},
        {Acc::Register::INT1_IO_CTRL,
         Acc::Int1IoConf::Int1Out | Acc::Int1Od_t(Acc::Int1Od::PUSH_PULL) |
             Acc::Int1Lvl_t(Acc::Int1Lvl::ACTIVE_LOW)},
        {Acc::Register::INT_MAP_DATA, Acc::IntMapData::INT1_DRDY}};

    for (size_t i = 0; i < MODM_ARRAY_SIZE(bmiAccRegData); i++)
    {
        bmi088AccWriteSingleReg(bmiAccRegData[i].reg, bmiAccRegData[i].value);
        modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

        uint8_t val = bmi088AccReadSingleReg(bmiAccRegData[i].reg);
        modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

        if (val != bmiAccRegData[i].value.value)
        {
            RAISE_ERROR(drivers, "bmi088 acc config failed");
            return;
        }
    }
}

void Bmi088::initializeGyro()
{
    // read GYRO_CHIP_ID to start SPI communication
    bmi088GyroReadSingleReg(Gyro::Register::GYRO_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    bmi088GyroReadSingleReg(Gyro::Register::GYRO_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    // reset gyro
    bmi088GyroWriteSingleReg(Gyro::Register::GYRO_SOFTRESET, Gyro::GyroSoftreset::RESET_SENSOR);
    modm::delay_us(BMI088_COMM_LONG_WAIT_TIME);

    // check communication normal after reset
    bmi088GyroReadSingleReg(Gyro::Register::GYRO_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    uint8_t res = bmi088GyroReadSingleReg(Gyro::Register::GYRO_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    if (res != Gyro::GYRO_CHIP_ID)
    {
        RAISE_ERROR(drivers, "bmi088 gyro init failed");
    }

    struct GyroTuple
    {
        Gyro::Register reg;
        Gyro::Registers_t value;
    };

    GyroTuple bmiGyroRegData[] = {
        {Gyro::Register::GYRO_RANGE, Gyro::GyroRange::DPS2000},
        {Gyro::Register::GYRO_BANDWIDTH, Gyro::GyroBandwidth::ODR1000_BANDWIDTH116},
        {Gyro::Register::GYRO_LPM1, Gyro::GyroLpm1::PWRMODE_NORMAL},
        {Gyro::Register::GYRO_INT_CTRL, Gyro::EnableNewDataInt_t(Gyro::EnableNewDataInt::ENABLED)},
        {Gyro::Register::INT3_INT4_IO_CONF,
         Gyro::Int3Od_t(Gyro::Int3Od::PUSH_PULL) | Gyro::Int3Lvl_t(Gyro::Int3Lvl::ACTIVE_LOW)},
        {Gyro::Register::INT3_INT4_IO_MAP, Gyro::Int3Int4IoMap::DATA_READY_INT3}};

    for (size_t i = 0; i < MODM_ARRAY_SIZE(bmiGyroRegData); i++)
    {
        bmi088GyroWriteSingleReg(bmiGyroRegData[i].reg, bmiGyroRegData[i].value);
        modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

        uint8_t val = bmi088GyroReadSingleReg(bmiGyroRegData[i].reg);
        modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

        if (val != bmiGyroRegData[i].value.value)
        {
            RAISE_ERROR(drivers, "bmi088 acc config failed");
            return;
        }
    }
}

bool Bmi088::run() { return false; }

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

#define LITTLE_ENDIAN_INT16_TO_FLOAT(buff) \
    (static_cast<float>(static_cast<int16_t>((*(buff) << 8) | *(buff + 1))))

void Bmi088::periodicIMUUpdate()
{
    uint8_t txBuff[6] = {};
    uint8_t rxBuff[6] = {};

    bmi088AccReadMultiReg(Acc::Register::ACC_X_LSB, rxBuff, txBuff, 6);
    data.acc[ImuData::Coordinate::X] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    data.acc[ImuData::Coordinate::Y] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    data.acc[ImuData::Coordinate::Z] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    bmi088GyroReadMultiReg(Gyro::Register::RATE_X_LSB, rxBuff, txBuff, 6);
    data.gyro[ImuData::Coordinate::X] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    data.gyro[ImuData::Coordinate::Y] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    data.gyro[ImuData::Coordinate::Z] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    bmi088AccReadMultiReg(Acc::Register::TEMP_LSB, rxBuff, txBuff, 2);
    int16_t parsedTemp = parseTemp(rxBuff[0], rxBuff[1]);
    data.temperature = parsedTemp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED)
    {
        mahonyAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
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
