#include "bmi088.hpp"

#include "tap/board/board.hpp"

#include "modm/math/units.hpp"
#include "bmi088_register_table.hpp"

using namespace modm::literals;
using namespace Board;

namespace sensors::bmi088
{
static inline void chipSelectGyro()
{
    ImuCS1Accel::set(false);
    ImuCS1Gyro::set(true);
}

static inline void chipSelectAcc()
{
    ImuCS1Accel::set(true);
    ImuCS1Gyro::set(false);
}

bool Bmi088::run()
{
#ifndef PLATFORM_HOSTED
    PT_BEGIN();
    while (true)
    {
        // PT_CALL(readGyroData());
        // PT_CALL(readAccData());
    }
    PT_END();
#else
    return false;
#endif
}

void Bmi088::init()
{
    ImuSpiMaster::connect<ImuMiso::Miso, ImuMosi::Mosi, ImuSck::Sck>();
    ImuSpiMaster::initialize<SystemClock, 10_MHz>();
    // using ImuCS1Accel = GpioA4;
    // using ImuCS1Gyro = GpioB0;
    // using ImuInt1Accel = GpioC4;
    // using ImuInt1Gyro = GpioC5;
}

// modm::ResumableResult<bool> Bmi088::readGyroData()
// {
//     RF_BEGIN(0);
//     // tx = 0;
//     // chipSelectGyro();
//     // RF_CALL(ImuSpiMaster::transfer(&tx, nullptr, 1));
//     // RF_CALL(
//     //     ImuSpiMaster::transfer(nullptr, reinterpret_cast<uint8_t *>(&rawGyro), sizeof(rawGyro)));
//     RF_END_RETURN(true);
// }

// modm::ResumableResult<bool> Bmi088::readAccData()
// {
//     RF_BEGIN(1);
//     // tx = static_cast<uint8_t>(Bmi088Acc::Register::ACC_X_LSB);
//     // chipSelectAcc();
//     // RF_CALL(ImuSpiMaster::transfer(&tx, nullptr, 1));
//     // RF_CALL(ImuSpiMaster::transfer(nullptr, reinterpret_cast<uint8_t *>(&rawAcc), sizeof(rawAcc)));
//     RF_END_RETURN(true);
// }

}  // namespace sensors::bmi088
