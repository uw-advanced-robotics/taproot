/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/bmi088/bmi088.hpp"
#include "tap/communication/sensors/imu/bmi088/bmi088_hal.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::sensors::imu::bmi088;

static void initializeBmi088(Bmi088 &bmi088)
{
    Bmi088Hal::expectAccReadSingleReg(Bmi088Data::Acc::ACC_CHIP_ID_VALUE);
    Bmi088Hal::expectAccReadSingleReg(Bmi088Data::Acc::ACC_CHIP_ID_VALUE);
    Bmi088Hal::expectAccWriteSingleReg();
    Bmi088Hal::expectAccReadSingleReg(Bmi088Data::Acc::ACC_CHIP_ID_VALUE);
    Bmi088Hal::expectAccReadSingleReg(Bmi088Data::Acc::ACC_CHIP_ID_VALUE);
    Bmi088Hal::expectAccWriteSingleReg();
    Bmi088Hal::expectAccReadSingleReg(171);  // acc config
    Bmi088Hal::expectAccWriteSingleReg();
    Bmi088Hal::expectAccReadSingleReg(0);  // acc range

    Bmi088Hal::expectGyroWriteSingleReg();
    Bmi088Hal::expectGyroReadSingleReg(Bmi088Data::Gyro::GYRO_CHIP_ID_VALUE);
    Bmi088Hal::expectGyroReadSingleReg(Bmi088Data::Gyro::GYRO_CHIP_ID_VALUE);
    Bmi088Hal::expectGyroWriteSingleReg();
    Bmi088Hal::expectGyroReadSingleReg(0);  // gyro range
    Bmi088Hal::expectGyroWriteSingleReg();
    Bmi088Hal::expectGyroReadSingleReg(130);  // gyro bandwidth
    Bmi088Hal::expectGyroWriteSingleReg();
    Bmi088Hal::expectGyroReadSingleReg(0);  // gyro powermode

    bmi088.initialize(1000, 0, 0);

    Bmi088Hal::clearData();
}

class Bmi088Test : public ::testing::Test
{
protected:
    tap::Drivers drivers;
    Bmi088 bmi088{&drivers};

    // packed‐structs for every test
    struct
    {
        int16_t x = 0x1234;
        int16_t y = 0x4321;
        int16_t z = 0x3214;
    } modm_packed accData;

    struct
    {
        int16_t x = 0x6789;
        int16_t y = 0x9876;
        int16_t z = 0x8769;
    } modm_packed gyroData;

    void SetUp() override
    {
        // common initialization
        initializeBmi088(bmi088);

        // common stubbing of the multi‐reads
        Bmi088Hal::expectAccMultiRead(reinterpret_cast<uint8_t *>(&accData), sizeof(accData));
        Bmi088Hal::expectGyroMultiRead(reinterpret_cast<uint8_t *>(&gyroData), sizeof(gyroData));
    }

    void expectTransform(
        float roll,
        float pitch,
        float yaw,
        float expectedGx,
        float expectedGy,
        float expectedGz,
        float expectedAx,
        float expectedAy,
        float expectedAz)
    {
        bmi088.setMountingTransform(
            tap::algorithms::transforms::Transform(0, 0, 0, roll, pitch, yaw));
        bmi088.read();
        bmi088.periodicIMUUpdate();

        static constexpr float EPS = 1E-3;
        EXPECT_NEAR(bmi088.getGx(), expectedGx * Bmi088::GYRO_RAD_PER_S_PER_GYRO_COUNT, EPS);
        EXPECT_NEAR(bmi088.getGy(), expectedGy * Bmi088::GYRO_RAD_PER_S_PER_GYRO_COUNT, EPS);
        EXPECT_NEAR(bmi088.getGz(), expectedGz * Bmi088::GYRO_RAD_PER_S_PER_GYRO_COUNT, EPS);
        EXPECT_NEAR(bmi088.getAx(), expectedAx * Bmi088::ACC_G_PER_ACC_COUNT, EPS);
        EXPECT_NEAR(bmi088.getAy(), expectedAy * Bmi088::ACC_G_PER_ACC_COUNT, EPS);
        EXPECT_NEAR(bmi088.getAz(), expectedAz * Bmi088::ACC_G_PER_ACC_COUNT, EPS);
    }
};

TEST_F(Bmi088Test, periodicIMUUpdate_initialize_called_no_errors)
{
    bmi088.read();
    bmi088.periodicIMUUpdate();
    EXPECT_EQ(Bmi088::ImuState::IMU_NOT_CALIBRATED, bmi088.getImuState());
}

TEST_F(Bmi088Test, periodicIMUUpdate_gyro_acc_temp_data_parsed_properly)
{
    expectTransform(0, 0, 0, gyroData.x, gyroData.y, gyroData.z, accData.x, accData.y, accData.z);
}

TEST_F(Bmi088Test, mounting_transform_90_deg_yaw)
{
    // x -> y, y -> -x, z -> z.

    // clang-format off
    expectTransform(
        0, 0, M_PI_2,
        gyroData.y, -gyroData.x, gyroData.z,
        accData.y, -accData.x, accData.z);
    // clang-format on
}

TEST_F(Bmi088Test, mounting_transform_180_deg_yaw)
{
    // x -> -x. y -> -y. z -> z.

    // clang-format off
    expectTransform(
        0, 0, M_PI,
        -gyroData.x, -gyroData.y, gyroData.z,
        -accData.x, -accData.y, accData.z);
    // clang-format on
}

TEST_F(Bmi088Test, mounting_transform_90_deg_pitch)
{
    // x -> -z, y -> y, z -> x.

    // clang-format off
    expectTransform(
        0, M_PI_2, 0,
        -gyroData.z, gyroData.y, gyroData.x,
        -accData.z, accData.y, accData.x);
    // clang-format on
}

TEST_F(Bmi088Test, mounting_transform_180_deg_pitch)
{
    // x -> -x, y -> y, z -> -z.

    // clang-format off
    expectTransform(
        0, M_PI, 0,
        -gyroData.x, gyroData.y, -gyroData.z,
        -accData.x, accData.y, -accData.z);
    // clang-format on
}

TEST_F(Bmi088Test, mounting_transform_90_deg_roll)
{
    // x -> x, y -> z, z -> -y.

    // clang-format off
    expectTransform(
        M_PI_2, 0, 0,
        gyroData.x, gyroData.z, -gyroData.y,
        accData.x, accData.z, -accData.y);
    // clang-format on
}

TEST_F(Bmi088Test, mounting_transform_180_deg_roll)
{
    // x -> x, y -> -y, z -> -z.

    // clang-format off
    expectTransform(
        M_PI, 0, 0,
        gyroData.x, -gyroData.y, -gyroData.z,
        accData.x, -accData.y, -accData.z);
    // clang-format on
}
