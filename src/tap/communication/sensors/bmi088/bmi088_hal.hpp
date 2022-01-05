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

#ifndef BMI088_HAL_HPP_
#define BMI088_HAL_HPP_

#include "tap/board/board.hpp"

#include "bmi088_data.hpp"
namespace tap::sensors::bmi088
{
inline void chipSelectAccelLow() { Board::ImuCS1Accel::set(false); }
inline void chipSelectAccelHigh() { Board::ImuCS1Accel::set(true); }
inline void chipSelectGyroLow() { Board::ImuCS1Gyro::set(false); }
inline void chipSelectGyroHigh() { Board::ImuCS1Gyro::set(true); }

inline uint8_t bmi088ReadWriteByte(uint8_t reg)
{
#ifndef PLATFORM_HOSTED
    uint8_t rx;
    Board::ImuSpiMaster::transferBlocking(&reg, &rx, 1);
    return rx;
#else
    return 0;
#endif
}

inline void bmi088WriteSingleReg(uint8_t reg, uint8_t data)
{
    bmi088ReadWriteByte(reg);
    bmi088ReadWriteByte(data);
}

inline uint8_t bmi088ReadSingleReg(uint8_t reg)
{
    bmi088ReadWriteByte(reg | Bmi088Data::BMI088_READ_BIT);
    return bmi088ReadWriteByte(0x55);
}

inline void bmi088ReadMultiReg(uint8_t reg, uint8_t *rxBuff, uint8_t *txBuff, uint8_t len)
{
    uint8_t tx = reg | Bmi088Data::BMI088_READ_BIT;
    uint8_t rx = 0;

    txBuff[0] = tx;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(txBuff, rxBuff, len);
}

inline void bmi088AccWriteSingleReg(
    Bmi088Data::Acc::Register reg,
    Bmi088Data::Acc::Registers_t data)
{
    chipSelectAccelHigh();
    bmi088WriteSingleReg(static_cast<uint8_t>(reg), data.value);
    chipSelectAccelLow();
}

inline uint8_t bmi088AccReadSingleReg(Bmi088Data::Acc::Register reg)
{
    chipSelectAccelHigh();
    uint8_t res = bmi088ReadSingleReg(static_cast<uint8_t>(reg));
    chipSelectAccelLow();
    return res;
}

inline void bmi088AccReadMultiReg(
    Bmi088Data::Acc::Register reg,
    uint8_t *rxBuff,
    uint8_t *txBuff,
    uint8_t len)
{
    chipSelectAccelHigh();
    bmi088ReadMultiReg(static_cast<uint8_t>(reg), rxBuff, txBuff, len);
    chipSelectAccelLow();
}

inline void bmi088GyroWriteSingleReg(
    Bmi088Data::Gyro::Register reg,
    Bmi088Data::Gyro::Registers_t data)
{
    chipSelectGyroHigh();
    bmi088WriteSingleReg(static_cast<uint8_t>(reg), data.value);
    chipSelectGyroLow();
}

inline uint8_t bmi088GyroReadSingleReg(Bmi088Data::Gyro::Register reg)
{
    chipSelectGyroHigh();
    uint8_t res = bmi088ReadSingleReg(static_cast<uint8_t>(reg));
    chipSelectGyroLow();
    return res;
}

inline void bmi088GyroReadMultiReg(
    Bmi088Data::Gyro::Register reg,
    uint8_t *rxBuff,
    uint8_t *txBuff,
    uint8_t len)
{
    chipSelectGyroHigh();
    bmi088ReadMultiReg(static_cast<uint8_t>(reg), rxBuff, txBuff, len);
    chipSelectGyroLow();
}
}  // namespace tap::sensors::bmi088

#endif  // BMI088_HAL_HPP_
