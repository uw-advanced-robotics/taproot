#include "bno055_interface.hpp"

using namespace modm::literals;

namespace tap
{
namespace sensors
{
Bno055Interface::Bno055Interface() : data(), unusedData(), imu(unusedData, BNO055_ADDR) {}

void Bno055Interface::initialize()
{
    Board::Bno055I2CMaster::connect<Board::Bno055I2CMasterScl::Scl, Board::Bno055I2CMasterSda::Sda>(
        modm::I2cMaster::PullUps::Internal);
    Board::Bno055I2CMaster::initialize<Board::SystemClock, 100_kHz>();
}

bool Bno055Interface::update()
{
    PT_BEGIN();

    // ping the device until it responds
    while (true)
    {
        // we wait until the device started
        if (PT_CALL(imu.ping()))
        {
            break;
        }
        PT_WAIT_UNTIL(timer.execute());
    }

    while (true)
    {
        if (PT_CALL(imu.configure(modm::bno055::OperationMode::AccGyro)))
        {
            break;
        }

        PT_WAIT_UNTIL(timer.execute());
    }

    while (true)
    {
        // Read acceleration/gyroscope data
        PT_WAIT_UNTIL(timer.execute());
        PT_CALL(imu.readRegister(
            modm::bno055::Register::ACCEL_DATA_X_LSB,
            reinterpret_cast<uint8_t *>(&data.acceleration),
            sizeof(data.acceleration)));
        PT_CALL(imu.readRegister(
            modm::bno055::Register::GYRO_DATA_X_LSB,
            reinterpret_cast<uint8_t *>(&data.gyroscope),
            sizeof(data.gyroscope)));
    }

    PT_END();
}
}  // namespace sensors
}  // namespace tap
