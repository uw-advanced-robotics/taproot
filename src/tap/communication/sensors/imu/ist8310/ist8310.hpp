/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_IST8310_HPP_
#define TAPROOT_IST8310_HPP_

#include "tap/board/board.hpp"

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"

#include "ist8310_config.hpp"
#include "ist8310_reg.hpp"

namespace
{
static uint8_t txBuffer[1] = {};
static uint8_t rxBuffer[IST8310_DATA_LENGTH] = {};
}

namespace tap::communication::sensors::imu::ist8310
{
/**
 * A class made for interfacing with the Robomaster type C board IST8310 magnetometer.
 *
 * To use this class, call initialize() once. Updating of the magnetometer is handled automatically
 * through the interrupt pin. Use the getter methods to access the data.
 */
class IST8310 : public modm::I2cDevice<Board::IST8310I2cMaster>, ::modm::pt::Protothread
{
public:
    IST8310();

    // Initializes device on the I2C bus
    void initialize();
    // Reads data from the device
    void update();

    inline float getMx() const { return x; }

    inline float getMy() const { return y; }

    inline float getMz() const { return z; }

private:
    float x, y, z;

    inline modm::ResumableResult<bool> readRegister(uint8_t reg, int len){
        RF_BEGIN();

        if(len > IST8310_DATA_LENGTH){
            len = IST8310_DATA_LENGTH;
        }

        txBuffer[0] = reg;
        while(!transaction.configureWriteRead(txBuffer, 1, rxBuffer, len));

        RF_END_RETURN_CALL(this->runTransaction());
    }
};

}  // namespace tap::communication::sensors::imu::ist8310
#endif
