#ifndef BMI055_REGISTER_TABLE_HPP_
#define BMI055_REGISTER_TABLE_HPP_

// accelerometer

#include "modm/architecture/interface/register.hpp"

/**
 * See
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 */
struct Bmi088
{
    enum class Register : uint8_t
    {
        ACC_SOFTRESET = 0x7e,
        ACC_PWR_CTRL = 0x7d,
        ACC_PWR_CONF = 0x7c,
        ACC_SELF_TEST = 0x6d,
        INT_MAP_DATA = 0x58,
        INT2_IO_CTRL = 0x54,
        INT1_IO_CTRL = 0x53,
        FIFO_CONFIG_1 = 0x49,
        FIFO_CONFIG_0 = 0x48,
        FIFO_WTM_1 = 0x47,
        FIFO_WTM_0 = 0x46,
        FIFO_DOWNS = 0x45,
        ACC_RANGE = 0x41,
        ACC_CONF = 0x40,
        FIFO_DATA = 0x26,
        FIFO_LENGTH_1 = 0x25,
        FIFO_LENGTH_0 = 0x24,
        TEMP_LSB = 0x23,
        TEMP_MSB = 0x22,
        ACC_INT_STAT_1 = 0x1d,
        SENSORTIME_2 = 0x1a,
        SENSORTIME_1 = 0x19,
        SENSORTIME_0 = 0x18,
        ACC_Z_MSB = 0x17,
        ACC_Z_LSB = 0x16,
        ACC_Y_MSB = 0x15,
        ACC_Y_LSB = 0x14,
        ACC_X_MSB = 0x13,
        ACC_X_LSB = 0x12,
        ACC_STATUS = 0x03,
        ACC_ERR_REG = 0x02,
        ACC_CHIP_ID = 0x01,
    };

    static constexpr uint8_t ACC_CHIP_ID = 0x1e;

    enum class AccErr
    {
        ERROR_CODE = modm::Bit2 | modm::Bit3 | modm::Bit4,
        FATAL_ERR = modm::Bit0,
    };
    MODM_FLAGS8(AccErr);

    enum class AccStatus
    {
        ACC_DRDY = modm::Bit7,
    };
    MODM_FLAGS8(AccStatus);
};

#endif  // BMI055_REGISTER_TABLE_HPP_
