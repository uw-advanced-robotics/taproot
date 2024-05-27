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

#ifndef TAPROOT_IST8310_REG_HPP_
#define TAPROOT_IST8310_REG_HPP_

namespace tap::comm::sensors::imu::ist8310
{
#define IST8310_WHO_AM_I 0x00

#define IST8310_STATUS_REGISTER1 0x02

#define IST8310_X_LOW_BYTE 0x03
#define IST8310_X_HIGH_BYTE 0x04
#define IST8310_Y_LOW_BYTE 0x05
#define IST8310_Y_HIGH_BYTE 0x06
#define IST8310_Z_LOW_BYTE 0x07
#define IST8310_Z_HIGH_BYTE 0x08

#define IST8310_STATUS_REGISTER2 0x09

#define IST8310_CONTROL_REGISTER1 0x0A
#define IST8310_CONTROL_REGISTER2 0x0B

#define IST8310_SELF_TEST_REGISTER 0x0C

#define IST8310_TEMPERATURE_LOW_BYTE 0x1C
#define IST8310_TEMPERATURE_HIGH_BYTE 0x1D

#define IST8310_AVERAGE_CONTROL_REGISTER 0x41
#define IST8310_PULSE_DURATION_CONTROL_REGISTER 0x42
}  // namespace tap::comm::sensors::imu::ist8310

#endif  // TAPROOT_IST8310_REG_HPP_
