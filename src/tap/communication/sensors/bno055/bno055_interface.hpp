#ifndef BNO055_INTERFACE_HPP_
#define BNO055_INTERFACE_HPP_

#include <modm/driver/inertial/bno055.hpp>
#include <modm/processing.hpp>

#include "tap/board/board.hpp"
#include "tap/util_macros.hpp"

namespace tap
{
namespace sensors
{
class Bno055Interface : public modm::pt::Protothread
{
public:
    static constexpr uint8_t BNO055_ADDR = 0x28;

    Bno055Interface();

    mockable void initialize();

    mockable bool update();

private:
    struct RawData
    {
        int16_t acceleration[3];
        int16_t gyroscope[3];
    };

    RawData data;
    modm::ShortPeriodicTimer timer{500};
    modm::bno055::Data unusedData;
    modm::Bno055<Board::Bno055I2CMaster> imu;
};
}  // namespace sensors
}  // namespace tap

#endif  // BNO055_INTERFACE_HPP_
