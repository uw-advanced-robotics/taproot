#include "bmi088.hpp"

#include "tap/architecture/endianness_wrappers.hpp"
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

static inline void bmi088ReadMultiReg(uint8_t reg, uint8_t *rxBuff, uint8_t *txBuff, uint8_t len)
{
    uint8_t tx = reg | Bmi088::BMI088_READ_BIT;
    uint8_t rx = 0;

    txBuff[0] = tx;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(txBuff, rxBuff, len);
}

static inline void bmi088AccWriteSingleReg(
    Bmi088Data::Acc::Register reg,
    Bmi088Data::Acc::Registers_t data)
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

static inline void bmi088AccReadMultiReg(
    Bmi088Data::Acc::Register reg,
    uint8_t *rxBuff,
    uint8_t *txBuff,
    uint8_t len)
{
    chipSelectAccelHigh();
    bmi088ReadMultiReg(static_cast<uint8_t>(reg), rxBuff, txBuff, len);
    chipSelectAccelLow();
}

static inline void bmi088GyroWriteSingleReg(
    Bmi088Data::Gyro::Register reg,
    Bmi088Data::Gyro::Registers_t data)
{
    chipSelectGyroHigh();
    bmi088WriteSingleReg(static_cast<uint8_t>(reg), data.value);
    chipSelectGyroLow();
}

static inline uint8_t bmi088GyroReadSingleReg(Bmi088Data::Gyro::Register reg)
{
    chipSelectGyroHigh();
    uint8_t res = bmi088ReadSingleReg(static_cast<uint8_t>(reg));
    chipSelectGyroLow();
    return res;
}

static inline void bmi088GyroReadMultiReg(
    Bmi088Data::Gyro::Register reg,
    uint8_t *rxBuff,
    uint8_t *txBuff,
    uint8_t len)
{
    chipSelectGyroHigh();
    bmi088ReadMultiReg(static_cast<uint8_t>(reg), rxBuff, txBuff, len);
    chipSelectGyroLow();
}

Bmi088::Bmi088(tap::Drivers *drivers) : drivers(drivers), imuHeater(drivers) {}

void Bmi088::initiailze()
{
    ImuSpiMaster::connect<ImuMiso::Miso, ImuMosi::Mosi, ImuSck::Sck>();
    ImuSpiMaster::initialize<SystemClock, 10_MHz>();

    modm::delay_ms(1);

    initializeAcc();
    initializeGyro();

    imuHeater.initialize();

    ready = true;
}

void Bmi088::initializeAcc()
{
    // read ACC_CHIP_ID to start SPI communication
    bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    // acc softreset
    bmi088AccWriteSingleReg(Acc::Register::ACC_SOFTRESET, Acc::AccSoftreset::ACC_SOFTRESET_VAL);
    modm::delay_us(BMI088_COMM_LONG_WAIT_TIME);

    // check communication is normal after reset
    bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    uint8_t res = bmi088AccReadSingleReg(Acc::Register::ACC_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    // check chip ID
    if (res != Acc::ACC_CHIP_ID)
    {
        RAISE_ERROR(drivers, "bmi088 gyro init failed");
        return;
    }

    // set acc sensor config and check
    struct AccTuple
    {
        Acc::Register reg;
        Acc::Registers_t value;
    };

    AccTuple bmiAccRegData[] = {
        {Acc::Register::ACC_PWR_CTRL, Acc::AccPwrConf(Acc::AccEnable::ACCELEROMETER_ON)},
        {Acc::Register::ACC_PWR_CONF, Acc::AccPwrCtrl(Acc::AccPwrSave::ACTIVE_MODE)},
        {Acc::Register::ACC_CONF,
         Acc::AccConf(Acc::AccBandwidth::NORMAL) |
             Acc::AccConf(Acc::AccOutputRate::Hz800)},  // potentially also | 0x80????
        {Acc::Register::ACC_RANGE, Acc::AccRange(Acc::AccRangeCtrl::G3)},
        {Acc::Register::INT1_IO_CTRL,
         Acc::Int1IoConf::Int1Out | Acc::Int1IoConf(Acc::Int1Od::PUSH_PULL) |
             Acc::Int1IoConf(Acc::Int1Lvl::ACTIVE_LOW)},
        {Acc::Register::INT_MAP_DATA, Acc::IntMapData::INT1_DRDY}};

    for (size_t i = 0; i < MODM_ARRAY_SIZE(bmiAccRegData); i++)
    {
        bmi088AccWriteSingleReg(bmiAccRegData[i].reg, bmiAccRegData[i].value);
        modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

        uint8_t val = bmi088AccReadSingleReg(bmiAccRegData[i].reg);
        modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

        if (val != bmiAccRegData[i].value.value)
        {
            RAISE_ERROR(drivers, "bmi088 acc config failed");
            return;
        }
    }
}

void Bmi088::initializeGyro()
{
    // read GYRO_CHIP_ID to start SPI communication
    bmi088GyroReadSingleReg(Gyro::Register::GYRO_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    bmi088GyroReadSingleReg(Gyro::Register::GYRO_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    // reset gyro
    bmi088GyroWriteSingleReg(Gyro::Register::GYRO_SOFTRESET, Gyro::GyroSoftreset::RESET_SENSOR);
    modm::delay_us(BMI088_COMM_LONG_WAIT_TIME);

    // check communication normal after reset
    bmi088GyroReadSingleReg(Gyro::Register::GYRO_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);
    uint8_t res = bmi088GyroReadSingleReg(Gyro::Register::GYRO_CHIP_ID);
    modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

    if (res != Gyro::GYRO_CHIP_ID)
    {
        RAISE_ERROR(drivers, "bmi088 gyro init failed");
    }

    struct GyroTuple
    {
        Gyro::Register reg;
        Gyro::Registers_t value;
    };

    GyroTuple bmiGyroRegData[] = {
        {Gyro::Register::GYRO_RANGE, Gyro::GyroRange(Gyro::GyroRangeCtrl::DPS2000)},
        {Gyro::Register::GYRO_BANDWIDTH, Gyro::GyroBandwidth(Gyro::GyroBw::ODR1000_BANDWIDTH116)},
        {Gyro::Register::GYRO_LPM1, Gyro::GyroLpm1(Gyro::GyroPm::PWRMODE_NORMAL)},
        {Gyro::Register::GYRO_INT_CTRL, Gyro::GyroIntCtrl(Gyro::EnableNewDataInt::ENABLED)},
        {Gyro::Register::INT3_INT4_IO_CONF,
         Gyro::Int3Int4IoConf(Gyro::Int3Od::PUSH_PULL) |
             Gyro::Int3Int4IoConf(Gyro::Int3Lvl::ACTIVE_LOW)},
        {Gyro::Register::INT3_INT4_IO_MAP, Gyro::Int3Int4IoMap::DATA_READY_INT3}};

    for (size_t i = 0; i < MODM_ARRAY_SIZE(bmiGyroRegData); i++)
    {
        bmi088GyroWriteSingleReg(bmiGyroRegData[i].reg, bmiGyroRegData[i].value);
        modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

        uint8_t val = bmi088GyroReadSingleReg(bmiGyroRegData[i].reg);
        modm::delay_us(BMI088_COMM_WAIT_SENSOR_TIME);

        if (val != bmiGyroRegData[i].value.value)
        {
            RAISE_ERROR(drivers, "bmi088 acc config failed");
            return;
        }
    }
}

bool Bmi088::run() { return false; }

static inline int16_t parseTemp(uint8_t tempMsb, uint8_t tempLsb)
{
    uint16_t temp = (static_cast<uint16_t>(tempMsb) * 8) + (static_cast<uint16_t>(tempLsb) / 32);

    if (temp > 1023)
    {
        return static_cast<int16_t>(temp) - 2048;
    }
    else
    {
        return static_cast<int16_t>(temp);
    }
}

#define LITTLE_ENDIAN_INT16_TO_FLOAT(buff) \
    (static_cast<float>(static_cast<int16_t>((*(buff) << 8) | *(buff + 1))))

void Bmi088::periodicIMUUpdate()
{
    uint8_t txBuff[6] = {};
    uint8_t rxBuff[6] = {};

    bmi088AccReadMultiReg(Acc::Register::ACC_X_LSB, rxBuff, txBuff, 6);
    data.acc.x = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    data.acc.y = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    data.acc.z = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    bmi088GyroReadMultiReg(Gyro::Register::RATE_X_LSB, rxBuff, txBuff, 6);
    data.gyro.x = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    data.gyro.y = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    data.gyro.z = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    bmi088AccReadMultiReg(Acc::Register::TEMP_LSB, rxBuff, txBuff, 2);
    int16_t parsedTemp = parseTemp(rxBuff[0], rxBuff[1]);
    data.temperature = parsedTemp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    mahonyAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());

    imuHeater.runTemperatureController(data.temperature);
}

}  // namespace tap::sensors::bmi088
