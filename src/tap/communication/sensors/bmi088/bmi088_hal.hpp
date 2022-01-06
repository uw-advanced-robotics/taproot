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
class Bmi088Hal
{
private:
    static inline void chipSelectAccelLow()
    {
        Board::ImuCS1Accel::setOutput(modm::GpioOutput::Low);
    }
    static inline void chipSelectAccelHigh()
    {
        Board::ImuCS1Accel::setOutput(modm::GpioOutput::High);
    }
    static inline void chipSelectGyroLow() { Board::ImuCS1Gyro::setOutput(modm::GpioOutput::Low); }
    static inline void chipSelectGyroHigh()
    {
        Board::ImuCS1Gyro::setOutput(modm::GpioOutput::High);
    }

    // uint8_t spiReadRegister(uint8_t reg)
    // {
    // #ifdef PLATFORM_HOSTED
    //     UNUSED(reg);
    //     return 0;
    // #else
    //     mpuNssLow();
    //     uint8_t tx = reg | MPU6500_READ_BIT;
    //     uint8_t rx = 0;
    //     Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    //     Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    //     mpuNssHigh();
    //     return rx;
    // #endif
    // }

    static inline uint8_t bmi088ReadWriteByte(uint8_t tx)
    {
#ifndef PLATFORM_HOSTED
        uint8_t rx = 0;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        return rx;
#else
        return 0;
#endif
    }

    /**
     * Primitive for writing some data to a register reg to the bmi088
     */
    static inline void bmi088WriteSingleReg(uint8_t reg, uint8_t data)
    {
        bmi088ReadWriteByte(reg & ~Bmi088Data::BMI088_READ_BIT);
        bmi088ReadWriteByte(data);
    }

    /**
     * Primitive for reading a single register's value from the bmi088
     */
    static inline uint8_t bmi088ReadSingleReg(uint8_t reg)
    {
        bmi088ReadWriteByte(reg | Bmi088Data::BMI088_READ_BIT);
        return bmi088ReadWriteByte(0x55);
    }

    static inline void bmi088ReadMultiReg(
        uint8_t reg,
        uint8_t *rxBuff,
        uint8_t len,
        bool readExtraByte)
    {
        bmi088ReadWriteByte(reg | Bmi088Data::BMI088_READ_BIT);

        if (readExtraByte) {
        bmi088ReadWriteByte(0x55);
        }

        while (len != 0)
        {
            *rxBuff = bmi088ReadWriteByte(0x55);
            rxBuff++;
            len--;
        }
    }

public:
    static inline void bmi088AccWriteSingleReg(
        Bmi088Data::Acc::Register reg,
        Bmi088Data::Acc::Registers_t data)
    {
        chipSelectAccelLow();
        bmi088WriteSingleReg(static_cast<uint8_t>(reg), data.value);
        chipSelectAccelHigh();
    }

    /**
     * From page 45 of the bmi088 datasheet: "In case of read operations, the SPI interface of the
     * accelerometer does not send the requested information directly after the master has sent the
     * corresponding register address, but sends a dummy byte first, whose content is not
     * predictable."
     *
     * Because of this, call `bmi088ReadWriteByte` one time extra to get valid data.
     */
    static inline uint8_t bmi088AccReadSingleReg(Bmi088Data::Acc::Register reg)
    {
        chipSelectAccelLow();
        bmi088ReadSingleReg(static_cast<uint8_t>(reg));
        uint8_t res = bmi088ReadWriteByte(0x55);
        chipSelectAccelHigh();
        return res;
    }

    static inline void bmi088AccReadMultiReg(
        Bmi088Data::Acc::Register reg,
        uint8_t *rxBuff,
        uint8_t len)
    {
        chipSelectAccelLow();
        bmi088ReadMultiReg(static_cast<uint8_t>(reg), rxBuff, len, true);
        chipSelectAccelHigh();
    }

    static inline void bmi088GyroWriteSingleReg(
        Bmi088Data::Gyro::Register reg,
        Bmi088Data::Gyro::Registers_t data)
    {
        chipSelectGyroLow();
        bmi088WriteSingleReg(static_cast<uint8_t>(reg), data.value);
        chipSelectGyroHigh();
    }

    static inline uint8_t bmi088GyroReadSingleReg(Bmi088Data::Gyro::Register reg)
    {
        chipSelectGyroLow();
        uint8_t res = bmi088ReadSingleReg(static_cast<uint8_t>(reg));
        chipSelectGyroHigh();
        return res;
    }

    static inline void bmi088GyroReadMultiReg(
        Bmi088Data::Gyro::Register reg,
        uint8_t *rxBuff,
        uint8_t len)
    {
        chipSelectGyroLow();
        bmi088ReadMultiReg(static_cast<uint8_t>(reg), rxBuff, len, false);
        chipSelectGyroHigh();
    }
};

}  // namespace tap::sensors::bmi088

#endif  // BMI088_HAL_HPP_
