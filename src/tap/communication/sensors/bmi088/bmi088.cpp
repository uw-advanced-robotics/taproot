#include "bmi088.hpp"

#include "tap/board/board.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/math/units.hpp"

#include "bmi088_register_table.hpp"

using namespace modm::literals;
using namespace Board;

namespace tap::sensors::bmi088
{
static inline void chipSelectAccelLow() { ImuCS1Accel::set(false); }
static inline void chipSelectAccelHigh() { ImuCS1Accel::set(true); }
static inline void chipSelectGyroLow() { ImuCS1Gyro::set(false); }
static inline void chipSelectGyroHigh() { ImuCS1Gyro::set(true); }

static inline uint8_t bmi088ReadWriteByte(uint8_t reg)
{
#ifndef PLATFORM_HOSTED
    uint8_t rx;
    ImuSpiMaster::transferBlocking(&reg, &rx, 1);
    return rx;
#else
    return 0;
#endif
}

static inline void bmi088WriteSingleReg(uint8_t reg, uint8_t data)
{
    bmi088ReadWriteByte(reg);
    bmi088ReadWriteByte(data);
}

static inline uint8_t bmi088ReadSingleReg(uint8_t reg)
{
    bmi088ReadWriteByte(reg | Bmi088::BMI088_READ_BIT);
    return bmi088ReadWriteByte(0x55);
}

static inline void bmi088AccWriteSingleReg(Bmi088Data::Acc::Register reg, Bmi088Data::Acc::Registers_t data)
{
    chipSelectAccelHigh();
    bmi088WriteSingleReg(static_cast<uint8_t>(reg), data.value);
    chipSelectAccelLow();
}

static inline uint8_t bmi088AccReadSingleReg(Bmi088Data::Acc::Register reg)
{
    chipSelectAccelHigh();
    uint8_t res = bmi088ReadSingleReg(static_cast<uint8_t>(reg));
    chipSelectAccelLow();
    return res;
}

static inline void bmi088GyroWriteSingleReg(uint8_t reg, uint8_t data)
{
    chipSelectGyroHigh();
    bmi088WriteSingleReg(reg, data);
    chipSelectGyroLow();
}

static inline uint8_t bmi088GyroReadSingleReg(uint8_t reg)
{
    chipSelectGyroHigh();
    uint8_t res = bmi088ReadSingleReg(reg);
    chipSelectGyroLow();
    return res;
}

void Bmi088::initiailze()
{
    ImuSpiMaster::connect<ImuMiso::Miso, ImuMosi::Mosi, ImuSck::Sck>();
    ImuSpiMaster::initialize<SystemClock, 10_MHz>();

    modm::delay_ms(1);

    initializeAcc();
    initializeGyro();
}

void Bmi088::initializeAcc()
{
    uint8_t res = 0;

    // read ACC_CHIP_ID to start SPI communication
    bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    // acc softreset
    bmi088AccWriteSingleReg(Acc::Register::ACC_SOFTRESET, Acc::ACC_SOFTRESET_VAL);
    modm::delay_us(BMI088_COMM_LONG_WAIT_TIME);

    // check communication after reset
    bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    res = bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    // check chip ID
    if (res != Acc::ACC_CHIP_ID)
    {
        // RAISE_ERROR(drivers, "bmi088 initialization failed");
        return;
    }

    // set acc sensor config and check


//     static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
//     {
//         {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
//         {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
//         {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
//         {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
//         {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
//         {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

// };


    
    bmi088AccWriteSingleReg(Acc::Register::ACC_PWR_CTRL, Acc::AccEnable::ACCELEROMETER_ON);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    // bmi088AccWriteSingleReg(, );
    // modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    // bmi088AccWriteSingleReg(, );
    // modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    // bmi088AccWriteSingleReg(, );
    // modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    // bmi088AccWriteSingleReg(, );
    // modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    // bmi088AccWriteSingleReg(, );
    // modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    // bmi088AccWriteSingleReg(, );
}

void Bmi088::initializeGyro() {}

bool Bmi088::run()
{
#ifndef PLATFORM_HOSTED
    PT_BEGIN();
    while (true)
    {
        // tx = 0;
        // chipSelectGyro();
        // RF_CALL(ImuSpiMaster::transfer(&tx, nullptr, 1));
        // RF_CALL(
        //     ImuSpiMaster::transfer(nullptr, reinterpret_cast<uint8_t *>(&rawGyro),
        //     sizeof(rawGyro)));

        //     // tx = static_cast<uint8_t>(Bmi088Acc::Register::ACC_X_LSB);
        //     // chipSelectAcc();
        //     // RF_CALL(ImuSpiMaster::transfer(&tx, nullptr, 1));
        //     // RF_CALL(ImuSpiMaster::transfer(nullptr, reinterpret_cast<uint8_t *>(&rawAcc),
        //     sizeof(rawAcc)));
    }
    PT_END();
#else
    return false;
#endif
}

void Bmi088::periodicIMUUpdate() {}

}  // namespace tap::sensors::bmi088
