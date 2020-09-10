/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef ENV_SIMULATOR
#include "mpu6500.hpp"

namespace aruwlib
{
namespace sensors
{
// initialize the imu and SPIbx
void Mpu6500::init() {}

// parse imu data from data buffer
void Mpu6500::read() {}

bool Mpu6500::initialized() const { return imuInitialized; }

// get accleration reading on x-axis
float Mpu6500::getAx() const { return 0; }

// get accleration reading on y-axis
float Mpu6500::getAy() const { return 0; }

// get acceleration reading on z-axis
float Mpu6500::getAz() const { return 0; }

// get gyro reading on x-axis
float Mpu6500::getGx() const { return 0; }

// get gyro reading on y-axis
float Mpu6500::getGy() const { return 0; }

// get gyro reading on z-axis (degrees per second)
float Mpu6500::getGz() const { return 0; }

// get temperature value in C
float Mpu6500::getTemp() const { return 0; }

float Mpu6500::getYaw() { return 0; }

float Mpu6500::getPitch() { return 0; }

float Mpu6500::getRoll() { return 0; }

float Mpu6500::getTiltAngle() { return 0; }
}  // namespace sensors

}  // namespace aruwlib
#endif
