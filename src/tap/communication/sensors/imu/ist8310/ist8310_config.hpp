/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_IST8310_CONFIG_HPP_
#define TAPROOT_IST8310_CONFIG_HPP_

namespace tap::communication::sensors::imu::ist8310
{
// Look at datahseet for more information about configuring the ist8310:
// https://intofpv.com/attachment.php?aid=8104

// TODO: According to https://github.com/Meta-Team/Meta-Embedded/blob/e19889a3b968af4eefb696caacda6aa13c32efd8/dev/interface/ahrs/ist8310_reg.h#L30 
// there is a 200hz mode for CTRL1 that is set by setting it to 0x0B, which is what the IST8308 can do.
// https://dokumen.tips/documents/ist8303-3d-magnetometer-sensor-datasheet-datasheetpdf-stat20x09-bit-description.html?page=16

// Address on the board
#define IST8310_IIC_ADDRESS 0x0E

// Expected return of WHO_AM_I
#define IST8310_DEVICE_ID 0x10

// Status Register 1
#define IST8310_DRDY (1 << 0)
#define IST8310_DOR (1 << 1)

// Status Register 2
#define IST8310_INTERRUPT_BIT (1 << 3)

// Control Register 1
#define IST8310_STAND_BY_MODE 0x00
#define IST8310_SINGLE_MEASUREMENT_MODE 0x01
#define IST8310_CONTINUOUS_MEASUREMENT_MODE_200HZ 0x0B // UIUC uses this in their code

// Control Register 2
#define IST8310_SOFT_RESET (1 << 0)
#define IST8310_DRDY_PIN_POLARITY_LOW (0 << 2)
#define IST8310_DRDY_PIN_POLARITY_HIGH (1 << 2)
#define IST8310_DATA_READY_ENABLE_CONTROL (1 << 3)

// Self Test Register
#define IST8310_SELF_TEST_ENABLE (1 << 6)

// Average Control Register
#define IST8310_Y__AXIS_NO_AVERAGE_SAMPLE (0b000 << 3)
#define IST8310_Y__AXIS_AVERAGE_2_SAMPLES (0b001 << 3)
#define IST8310_Y__AXIS_AVERAGE_4_SAMPLES (0b010 << 3) // Default
#define IST8310_Y__AXIS_AVERAGE_8_SAMPLES (0b011 << 3)
#define IST8310_Y__AXIS_AVERAGE_16_SAMPLES (0b100 << 3)

#define IST8310_XZ_AXIS_NO_AVERAGE_SAMPLE 0b000
#define IST8310_XZ_AXIS_AVERAGE_2_SAMPLES 0b001
#define IST8310_XZ_AXIS_AVERAGE_4_SAMPLES 0b010 // Default
#define IST8310_XZ_AXIS_AVERAGE_8_SAMPLES 0b011
#define IST8310_XZ_AXIS_AVERAGE_16_SAMPLES 0b100
// any other value results in no averaging

// Pulse Duration Control Register
#define IST8310_PULSE_DURATION_LONG (0b01 << 6)
#define IST8310_PULSE_DURATION_NORMAL (0b11 << 6)

// Configuration Data
#define IST8310_CONTROL_REGISTER1_DATA (IST8310_CONTINUOUS_MEASUREMENT_MODE_200HZ)
#define IST8310_CONTROL_REGISTER2_DATA (IST8310_DRDY_PIN_POLARITY_LOW | IST8310_DATA_READY_ENABLE_CONTROL)
#define IST8310_AVERAGE_CONTROL_REGISTER_DATA (IST8310_Y__AXIS_AVERAGE_16_SAMPLES | IST8310_XZ_AXIS_AVERAGE_16_SAMPLES)
#define IST8310_PULSE_DURATION_CONTROL_REGISTER_DATA (IST8310_PULSE_DURATION_NORMAL)

// Data transmit info
#define IST8310_DATA_LENGTH 6
#define IST8310_DATA_START_ADDRESS (IST8310_X_LOW_BYTE)

#define IST8310_SLOW_REFRESH_RATE_MS 6

}  // namespace tap::communication::sensors::imu::ist8310

#endif // TAPROOT_IST8310_CONFIG_HPP_
