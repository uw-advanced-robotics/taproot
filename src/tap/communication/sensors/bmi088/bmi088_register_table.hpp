#ifndef BMI055_REGISTER_TABLE_HPP_
#define BMI055_REGISTER_TABLE_HPP_

#include "modm/architecture/interface/register.hpp"

namespace sensors::bmi088
{
/**
 * See
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 */
struct Bmi088Data
{
    struct Gryo
    {
        enum class Register : uint8_t
        {
            GYRO_FIFO_DATA = 0x3f,
            GYRO_FIFO_CONFIG_1 = 0x3e,
            GYRO_CONFIG_0 = 0x3d,
            GYRO_SELF_TEST = 0x3c,
            GYRO_FIFO_EXT_INT_S = 0x34,
            GYRO_INT_CTRL = 0x15,
            GYRO_SOFTRESET = 0x14,
            GYRO_FIFO_WM_EN = 0x1e,
            INT3_INT4_IO_MAP = 0x18,
            INT3_INT4_IO_CONF = 0x16,
            GYRO_INT_STAT_1 = 0x0a,
            GYRO_Z_MSB = 0X07,
            GYRO_Z_LSB = 0X06,
            GYRO_Y_MSB = 0X05,
            GYRO_Y_LSB = 0X04,
            GYRO_X_MSB = 0X03,
            GYRO_X_LSB = 0X02,
            GYRO_CHIP_ID = 0x00,
        };

        static constexpr uint8_t GYRO_CHIP_ID = 0x0f;

        enum class GryoIntStat1 : uint8_t
        {
            GyroDrdy_Mask = modm::Bit7,
            FifoInt_Mask = modm::Bit4
        };
        MODM_FLAGS8(GryoIntStat1);

        enum class FifoStatus : uint8_t
        {
            FifoOverrun_Mask = modm::Bit7,
            FifoFrameCounter = modm::Bit0 | modm::Bit1 | modm::Bit2 | modm::Bit3 | modm::Bit4 |
                               modm::Bit5 | modm::Bit6
        };
        MODM_FLAGS8(FifoStatus);

        enum class GryoRange : uint8_t
        {
            DPS2000 = 0x00,
            DPS1000 = 0x01,
            DPS500 = 0x02,
            DPS250 = 0x03,
            DPS125 = 0x04
        };

        enum class GyroBandwidth : uint8_t
        {
            ODR2000_BANDWIDTH532 = 0x00,
            ODR2000_BANDWIDTH230 = 0x01,
            ODR1000_BANDWIDTH116 = 0x02,
            ODR400_BANDWIDTH47 = 0x03,
            ODR200_BANDWIDTH23 = 0x04,
            ODR100_BANDWIDTH12 = 0x05,
            ODR200_BANDWIDTH64 = 0x06,
            ODR100_BANDWIDTH32 = 0x07
        };

        enum class GyroLpm1 : uint8_t
        {
            PWRMODE_NORMAL = 0x00,
            PWRMODE_SUSPEND = 0x80,
            PWRMODE_DEEP_SUSPEND = 0x20
        };

        enum class GyroSoftreset : uint8_t
        {
            RESET_SENSOR = 0xb6
        };

        enum class GyroIntCtrl : uint8_t
        {
            EnableNewDataInt_Mask = modm::Bit7,
            EnableFifoInt_Mask = modm::Bit6
        };
        MODM_FLAGS8(GyroIntCtrl);

        enum class EnableNewDataInt : uint8_t
        {
            DISABLED = 0x00,
            ENABLED = 0x01
        };
        MODM_FLAGS_CONFIG(GyroIntCtrl, EnableNewDataInt);
        enum class EnableFifoInt : uint8_t
        {
            DISABLED = 0x00,
            ENABLED = 0x01
        };
        MODM_FLAGS_CONFIG(GyroIntCtrl, EnableFifoInt);

        enum class Int3Int4IoConf
        {
        };
        MODM_FLAGS8(Int3Int4IoConf);

        enum class Int3Int4IoMap
        {
        };
        MODM_FLAGS8(Int3Int4IoMap);

        enum class FifoWmEnable
        {
        };
        MODM_FLAGS8(FifoWmEnable);

        enum class FifoExtIntS
        {
        };
        MODM_FLAGS8(FifoExtIntS);

        enum class GyroSelfTest
        {
        };
        MODM_FLAGS8(GyroSelfTest);

        enum class FifoConfig0
        {
        };
        MODM_FLAGS8(FifoConfig0);

        enum class FifoConfig1
        {
        };
        MODM_FLAGS8(FifoConfig1);
    };
    struct Acc
    {
        enum class Register : uint8_t
        {
            ACC_CHIP_ID = 0x00,
            ACC_ERR_REG = 0x02,
            ACC_STATUS = 0x03,
            ACC_X_LSB = 0x12,
            ACC_X_MSB = 0x13,
            ACC_Y_LSB = 0x14,
            ACC_Y_MSB = 0x15,
            ACC_Z_LSB = 0x16,
            ACC_Z_MSB = 0x17,
            SENSORTIME_0 = 0x18,
            SENSORTIME_1 = 0x19,
            SENSORTIME_2 = 0x1a,
            ACC_INT_STAT_1 = 0x1d,
            TEMP_MSB = 0x22,
            TEMP_LSB = 0x23,
            FIFO_LENGTH_0 = 0x24,
            FIFO_LENGTH_1 = 0x25,
            FIFO_DATA = 0x26,
            ACC_CONF = 0x40,
            ACC_RANGE = 0x41,
            INT1_IO_CTRL = 0x53,
            INT2_IO_CTRL = 0x54,
            INT_MAP_DATA = 0x58,
            ACC_SELF_TEST = 0x6d,
            ACC_PWR_CONF = 0x7c,
            ACC_PWR_CTRL = 0x7d,
            ACC_SOFTRESET = 0x7e,
        };

        static constexpr uint8_t ACC_CHIP_ID = 0x1e;

        enum class AccErr : uint8_t
        {
            ERROR_CODE = modm::Bit2 | modm::Bit3 | modm::Bit4,
            FATAL_ERR = modm::Bit0,
        };
        MODM_FLAGS8(AccErr);

        enum class AccStatus : uint8_t
        {
            ACC_DRDY = modm::Bit7,
        };
        MODM_FLAGS8(AccStatus);

        enum class AccIntStat1 : uint8_t
        {
            ACC_DRDY = modm::Bit7,
        };
        MODM_FLAGS8(AccIntStat1);

        enum class AccConf : uint8_t
        {
            AccBandwidth_Mask = modm::Bit4 | modm::Bit5 | modm::Bit6 | modm::Bit7,
            AccOutputRate_Mask = modm::Bit0 | modm::Bit1 | modm::Bit2 | modm::Bit3,
        };
        MODM_FLAGS8(AccConf);

        /**
         * @see section 4.4.1 of the bmi088 datasheet.
         */
        enum class AccBandwidth : uint8_t
        {
            OSR4_OVERSAMPLING = 0x08,
            OSR2_OVERSAMPLING = 0x09,
            NORMAL = 0x0a
        };
        typedef modm::Configuration<AccConf_t, AccBandwidth, 0b1111, 4> AccBandwidth_t;  // Bit 4..7

        enum class AccOutputRate : uint8_t
        {
            Hz12_5 = 0x05,
            Hz25 = 0x06,
            Hz50 = 0x07,
            Hz100 = 0x08,
            Hz200 = 0x09,
            Hz400 = 0x0a,
            Hz800 = 0x0b,
            Hz1600 = 0x0c
        };
        MODM_FLAGS_CONFIG(AccConf, AccOutputRate);

        enum class AccRange : uint8_t
        {
            AccRangeCtrl_Mask = modm::Bit0 | modm::Bit1
        };
        MODM_FLAGS8(AccRange);

        enum class AccRangeCtrl : uint8_t
        {
            G3 = 0x0,
            G6 = 0x1,
            G12 = 0x2,
            G24 = 0x03,
        };
        MODM_FLAGS_CONFIG(AccRange, AccRangeCtrl);

        enum class Int1IoConf : uint8_t
        {
            Int1In_Mask = modm::Bit4,
            Int1Out_Mask = modm::Bit3,
            Int1Od_Mask = modm::Bit2,
            Int1Lvl_Mask = modm::Bit1
        };
        MODM_FLAGS8(Int1IoConf);

        enum class Int1Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1
        };
        typedef modm::Configuration<Int1IoConf_t, Int1Od, 0b1, 2> Int1Od_t;

        enum class Int1Lvl : uint8_t
        {
            ACTIVE_LOW = 0,
            ACTIVE_HIGH = 1
        };
        typedef modm::Configuration<Int1IoConf_t, Int1Lvl, 0b1, 1> Int1Lv1_t;

        enum class Int2IoConf : uint8_t
        {
            Int2Io_Mask = modm::Bit4,
            Int2Out_Mask = modm::Bit3,
            Int2Od_Mask = modm::Bit2,
            Int2Lvl_Mask = modm::Bit1
        };
        MODM_FLAGS8(Int2IoConf);

        enum class Int2Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1
        };
        typedef modm::Configuration<Int2IoConf_t, Int2Od, 0b1, 2> Int2Od_t;

        enum class Int2Lvl : uint8_t
        {
            ACTIVE_LOW = 0x00,
            ACTIVE_HIGH = 0x01
        };
        typedef modm::Configuration<Int2IoConf_t, Int2Lvl, 0b1, 1> Int2Lv1_t;

        enum class AccSelfTest : uint8_t
        {
            AccSelfTestCtrl_Mask = 255,
        };
        MODM_FLAGS8(AccSelfTest);

        enum class AccSelfTestCtrl : uint8_t
        {
            SELF_TEST_OFF = 0x00,
            POSITIVE_SELF_TEST_SIGNAL = 0x0d,
            NEGATIVE_SELF_TEST_SIGNAL = 0x09
        };
        MODM_FLAGS_CONFIG(AccSelfTest, AccSelfTestCtrl);

        enum class AccPwrConf : uint8_t
        {
            AccPwrSave_Mask = 255,
        };
        MODM_FLAGS8(AccPwrConf);

        enum class AccPwrSave : uint8_t
        {
            ACTIVE_MODE = 0x00,
            SUSPEND_MODE = 0x03
        };
        MODM_FLAGS_CONFIG(AccPwrConf, AccPwrSave);

        enum class AccPwrCtrl : uint8_t
        {
            AccEnable_Mask = 255,
        };
        MODM_FLAGS8(AccPwrCtrl);

        enum class AccEnable : uint8_t
        {
            ACCELEROMETER_OFF = 0X00,
            ACCELEROMETER_ON = 0X04
        };
        MODM_FLAGS_CONFIG(AccPwrCtrl, AccEnable);

        enum class AccSoftreset : uint8_t
        {
            RESET_SENSOR = 255,
        };
        MODM_FLAGS8(AccSoftreset);

        /** Writing this to the AccSoftreset register will perform a soft reset of the IMU */
        static constexpr uint8_t ACC_SOFTRESET_VAL = 0xb6;

        using Registers_t = modm::FlagsGroup<
            AccErr_t,
            AccStatus_t,
            AccIntStat1_t,
            AccConf_t,
            AccBandwidth_t,
            AccOutputRate_t,
            AccRangeCtrl_t,
            Int1IoConf_t,
            Int1Od_t,
            Int2IoConf_t,
            AccSelfTest_t,
            AccPwrConf_t,
            AccPwrCtrl_t,
            AccSoftreset_t>;
    };

    static constexpr float ACC_RANGE = 0;

    struct Data
    {
        /** @return accel data in mg */
        inline float getAccX() const
        {
            return raw.acc.x / 32768 * 1000 * powf(2, ACC_RANGE + 1) * 1.5f;
        }
        inline float getAccY() const
        {
            return raw.acc.y / 32768 * 1000 * powf(2, ACC_RANGE + 1) * 1.5f;
        }
        inline float getAccZ() const
        {
            return raw.acc.z / 32768 * 1000 * powf(2, ACC_RANGE + 1) * 1.5f;
        }

        /** @return temperature data in degrees C */
        inline float getTemp() const { return raw.rawTemp * 0.125 + 23; }

        /** @return  */

        struct
        {
            struct
            {
                int16_t x, y, z;
            } modm_packed acc;

            int16_t rawTemp;

            struct RawGyro
            {
                int16_t x, y, z;
            };
        } modm_packed raw;

        inline int16_t parseTemp(uint8_t tempMsb, uint8_t tempLsb)
        {
            uint16_t temp = (tempMsb * 8) + (tempLsb / 32);
            if (temp > 1023)
            {
                return static_cast<int16_t>(temp) - 2048;
            }
            else
            {
                return static_cast<int16_t>(temp);
            }
        }
    };
};

}  // namespace sensors::bmi088

#endif  // BMI055_REGISTER_TABLE_HPP_
