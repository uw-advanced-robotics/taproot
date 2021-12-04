#ifndef BMI055_REGISTER_TABLE_HPP_
#define BMI055_REGISTER_TABLE_HPP_

#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"

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
            GYRO_CHIP_ID = 0x00,
            RATE_X_LSB = 0X02,
            RATE_X_MSB = 0X03,
            RATE_Y_LSB = 0X04,
            RATE_Y_MSB = 0X05,
            RATE_Z_LSB = 0X06,
            RATE_Z_MSB = 0X07,
            GYRO_INT_STAT_1 = 0x0a,
            FIFO_STATUS = 0x0e,
            GYRO_RANGE = 0x0f,
            GYRO_BANDWIDTH = 0x10,
            GYRO_LPM1 = 0x11,
            GYRO_SOFTRESET = 0x14,
            GYRO_INT_CTRL = 0x15,
            INT3_INT4_IO_CONF = 0x16,
            INT3_INT4_IO_MAP = 0x18,
            FIFO_WM_EN = 0x1e,
            FIFO_EXT_INT_S = 0x34,
            GYRO_SELF_TEST = 0x3c,
            FIFO_CONFIG_0 = 0x3d,
            FIFO_CONFIG_1 = 0x3e,
            FIFO_DATA = 0x3f,
        };

        /** The id of the gyroscope that will is stored in address `GYRO_CHIP_ID`. */
        static constexpr uint8_t GYRO_CHIP_ID = 0x0f;

        enum class GryoIntStat1 : uint8_t
        {
            GYRO_DRDY = modm::Bit7,
            FIFO_INT = modm::Bit4,
        };
        MODM_FLAGS8(GryoIntStat1);

        enum class FifoStatus : uint8_t
        {
            FIFO_OVERRUN = modm::Bit7,
            FIFO_FRAME_COUNTER = static_cast<uint8_t>(~modm::Bit7),
        };
        MODM_FLAGS8(FifoStatus);

        enum class GyroRange : uint8_t
        {
            GyroRangeCtrl_Mask = 255,
        };
        MODM_FLAGS8(GyroRange);

        enum class GyroRangeCtrl : uint8_t
        {
            DPS2000 = 0x00,
            DPS1000 = 0x01,
            DPS500 = 0x02,
            DPS250 = 0x03,
            DPS125 = 0x04,
        };
        MODM_FLAGS_CONFIG(GyroRange, GyroRangeCtrl);

        enum class GyroBandwidth : uint8_t
        {
            GyroBw_Mask = 255,
        };
        MODM_FLAGS8(GyroBandwidth);

        enum class GyroBw : uint8_t
        {
            ODR2000_BANDWIDTH532 = 0x00,
            ODR2000_BANDWIDTH230 = 0x01,
            ODR1000_BANDWIDTH116 = 0x02,
            ODR400_BANDWIDTH47 = 0x03,
            ODR200_BANDWIDTH23 = 0x04,
            ODR100_BANDWIDTH12 = 0x05,
            ODR200_BANDWIDTH64 = 0x06,
            ODR100_BANDWIDTH32 = 0x07,
        };
        MODM_FLAGS_CONFIG(GyroBandwidth, GyroBw);

        enum class GyroLpm1
        {
            GyroPm_Mask = 255,
        };
        MODM_FLAGS8(GyroLpm1);

        enum class GyroPm : uint8_t
        {
            PWRMODE_NORMAL = 0x00,
            PWRMODE_SUSPEND = 0x80,
            PWRMODE_DEEP_SUSPEND = 0x20
        };
        MODM_FLAGS_CONFIG(GyroLpm1, GyroPm);

        enum class GyroSoftreset : uint8_t
        {
            RESET_SENSOR = 0xb6
        };

        enum class GyroIntCtrl : uint8_t
        {
            EnableNewDataInt_Mask = modm::Bit7,
            EnableFifoInt_Mask = modm::Bit6,
        };
        MODM_FLAGS8(GyroIntCtrl);

        enum class EnableNewDataInt : uint8_t
        {
            DISABLED = 0x00,
            ENABLED = 0x01,
        };
        MODM_FLAGS_CONFIG(GyroIntCtrl, EnableNewDataInt);

        enum class EnableFifoInt : uint8_t
        {
            DISABLED = 0x00,
            ENABLED = 0x01,
        };
        MODM_FLAGS_CONFIG(GyroIntCtrl, EnableFifoInt);

        enum class Int3Int4IoConf : uint8_t
        {
            Int4Od_Mask = modm::Bit3,
            Int4Lvl_Mask = modm::Bit2,
            Int3Od_Mask = modm::Bit1,
            Int3Lvl_Mask = modm::Bit0,
        };
        MODM_FLAGS8(Int3Int4IoConf);

        enum class Int4Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1,
        };
        MODM_FLAGS_CONFIG(Int3Int4IoConf, Int4Od);

        enum class Int4Lvl : uint8_t
        {
            ACTIVE_LOW = 0,
            ACTIVE_HIGH = 1,
        };
        MODM_FLAGS_CONFIG(Int3Int4IoConf, Int4Lvl);

        enum class Int3Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1,
        };
        MODM_FLAGS_CONFIG(Int3Int4IoConf, Int3Od);

        enum class Int3Lvl : uint8_t
        {
            ACTIVE_LOW = 0,
            ACTIVE_HIGH = 1,
        };
        MODM_FLAGS_CONFIG(Int3Int4IoConf, Int3Lvl);

        enum class Int3Int4IoMap : uint8_t
        {
            DATA_READY_INT4 = modm::Bit7,
            FIFO_INT4 = modm::Bit5,
            FIFO_INT3 = modm::Bit2,
            DATA_READY_INT3 = modm::Bit0,
        };
        MODM_FLAGS8(Int3Int4IoMap);

        enum class FifoWmEnable : uint8_t
        {
            FifoWmEnableCtrl_Mask = 255,
        };
        MODM_FLAGS8(FifoWmEnable);

        enum class FifoWmEnableCtrl : uint8_t
        {
            FIFO_WATERMARK_LVL_INT_DISABLED = 0x08,
            FIFO_WATERMARK_LVL_INT_ENABLED = 0x88,
        };
        MODM_FLAGS_CONFIG(FifoWmEnable, FifoWmEnableCtrl);

        enum class FifoExtIntS : uint8_t
        {
            ENABLE_EXTERNAL_FIFO_SYNCH_MODE = modm::Bit5,
            EXT_FIFO_S_SEL = modm::Bit4,
        };
        MODM_FLAGS8(FifoExtIntS);

        enum class GyroSelfTest : uint8_t
        {
            RATE_OK = modm::Bit4,
            BIST_FAIL = modm::Bit2,
            BIST_RDY = modm::Bit1,
            TRIG_BIST = modm::Bit0,
        };
        MODM_FLAGS8(GyroSelfTest);

        /**
         * defines the FIFO watermark level. An interrupt will be generated, when the number of
         * entries in the FIFO exceeds fifo_water_mark_level_trigger_retain<6:0>. Writing to this
         * register clears the FIFO buffer.
         */
        enum class FifoConfig0 : uint8_t
        {
            FIFO_WATER_MARK_LVL_TRIGGER_RETAIN = static_cast<uint8_t>(~modm::Bit7),
        };
        MODM_FLAGS8(FifoConfig0);

        enum class FifoConfig1 : uint8_t
        {
            FifoMode_Mask = 255,
        };
        MODM_FLAGS8(FifoConfig1);

        enum class FifoMode : uint8_t
        {
            FIFO = 0x40,
            STREAM = 0x80,
        };
        MODM_FLAGS_CONFIG(FifoConfig1, FifoMode);
    };

    struct Acc
    {
        /** List of register addresses for the bmi088's accelerometer */
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

        /** The id of the accelerometer that will is stored in address `ACC_CHIP_ID`. */
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

        /** @see section 4.4.1 of the bmi088 datasheet. */
        enum class AccBandwidth : uint8_t
        {
            OSR4_OVERSAMPLING = 0x08,
            OSR2_OVERSAMPLING = 0x09,
            NORMAL = 0x0a,
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
            Hz1600 = 0x0c,
        };
        MODM_FLAGS_CONFIG(AccConf, AccOutputRate);

        enum class AccRange : uint8_t
        {
            AccRangeCtrl_Mask = modm::Bit0 | modm::Bit1,
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
            Int1Lvl_Mask = modm::Bit1,
        };
        MODM_FLAGS8(Int1IoConf);

        enum class Int1Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1,
        };
        typedef modm::Configuration<Int1IoConf_t, Int1Od, 0b1, 2> Int1Od_t;

        enum class Int1Lvl : uint8_t
        {
            ACTIVE_LOW = 0,
            ACTIVE_HIGH = 1,
        };
        typedef modm::Configuration<Int1IoConf_t, Int1Lvl, 0b1, 1> Int1Lv1_t;

        enum class Int2IoConf : uint8_t
        {
            Int2Io_Mask = modm::Bit4,
            Int2Out_Mask = modm::Bit3,
            Int2Od_Mask = modm::Bit2,
            Int2Lvl_Mask = modm::Bit1,
        };
        MODM_FLAGS8(Int2IoConf);

        enum class Int2Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1,
        };
        typedef modm::Configuration<Int2IoConf_t, Int2Od, 0b1, 2> Int2Od_t;

        enum class Int2Lvl : uint8_t
        {
            ACTIVE_LOW = 0x00,
            ACTIVE_HIGH = 0x01,
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
            NEGATIVE_SELF_TEST_SIGNAL = 0x09,
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

    static constexpr Acc::AccRangeCtrl_t EXPECTED_ACC_RANGE =
        Acc::AccRangeCtrl_t(Acc::AccRangeCtrl::G12);

    struct Data
    {
        static constexpr float ACCEL_COUNTS_TO_MG =
            1000.0f * modm::pow(2, EXPECTED_ACC_RANGE.value + 1) * 1.5f / 32768.0f;

        static constexpr float LSB_D_PER_S_TO_D_PER_S = 16.384f;

        /** @return accel data in mg (m / s^2) */
        inline float getAccX() const { return raw.accX * ACCEL_COUNTS_TO_MG; }
        inline float getAccY() const { return raw.accY * ACCEL_COUNTS_TO_MG; }
        inline float getAccZ() const { return raw.accZ * ACCEL_COUNTS_TO_MG; }

        /** @return gyro data in deg/s */
        inline float getGyroX() const { return raw.gyroX / LSB_D_PER_S_TO_D_PER_S; }
        inline float getGyroY() const { return raw.gyroY / LSB_D_PER_S_TO_D_PER_S; }
        inline float getGyroZ() const { return raw.gyroZ / LSB_D_PER_S_TO_D_PER_S; }

        /** @return temperature data in degrees C */
        inline float getTemp() const { return raw.rawTemp * 0.125 + 23; }

        struct
        {
            int16_t accX, accY, accZ;
            int16_t gyroX, gyroY, gyroZ;
            int16_t rawTemp;
        } modm_packed raw;

        inline int16_t parseTemp(uint8_t tempMsb, uint8_t tempLsb)
        {
            uint16_t temp =
                (static_cast<uint16_t>(tempMsb) * 8) + (static_cast<uint16_t>(tempLsb) / 32);

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
