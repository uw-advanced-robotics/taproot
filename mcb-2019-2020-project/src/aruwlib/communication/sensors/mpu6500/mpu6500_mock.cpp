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
