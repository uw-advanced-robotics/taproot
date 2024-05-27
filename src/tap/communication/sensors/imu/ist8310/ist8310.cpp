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

#include "ist8310.hpp"

#include "tap/drivers.hpp"

IST8310::IST8310() : modm::I2cDevice<Board::IST8310I2cMaster>(IST8310_IIC_ADDRESS) {}

void IST8310::initialize() {}

void IST8310::update()
{
    while (!read());

    x = (int16_t)(rxBuffer[1] << 8 | rxBuffer[0]);
    y = (int16_t)(rxBuffer[3] << 8 | rxBuffer[2]);
    z = (int16_t)(rxBuffer[5] << 8 | rxBuffer[4]);
}

}  // namespace tap::communication::sensors::imu::ist8310
