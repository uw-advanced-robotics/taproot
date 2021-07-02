/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "aruwlib/architecture/endianness_wrappers.hpp"

TEST(EndiannessWrappersTest, ToLittleEndian)
{
    // 16 bit
    uint8_t data_output_16[2] = {};
    aruwlib::arch::convertToLittleEndian((uint16_t)0x1100, data_output_16);
    ASSERT_EQ(data_output_16[0], 0x00);
    ASSERT_EQ(data_output_16[1], 0x11);

    // 32 bit
    uint8_t data_output_32[4] = {};
    aruwlib::arch::convertToLittleEndian((uint32_t)0x33221100, data_output_32);
    ASSERT_EQ(data_output_32[0], 0x00);
    ASSERT_EQ(data_output_32[1], 0x11);
    ASSERT_EQ(data_output_32[2], 0x22);
    ASSERT_EQ(data_output_32[3], 0x33);

    // 64 bit
    uint8_t data_output_64[8] = {};
    aruwlib::arch::convertToLittleEndian((uint64_t)0x7766554433221100, data_output_64);
    ASSERT_EQ(data_output_64[0], 0x00);
    ASSERT_EQ(data_output_64[1], 0x11);
    ASSERT_EQ(data_output_64[2], 0x22);
    ASSERT_EQ(data_output_64[3], 0x33);
    ASSERT_EQ(data_output_64[4], 0x44);
    ASSERT_EQ(data_output_64[5], 0x55);
    ASSERT_EQ(data_output_64[6], 0x66);
    ASSERT_EQ(data_output_64[7], 0x77);
}

TEST(EndiannessWrappersTest, ToBigEndian)
{
    // 16 bit
    uint8_t data_output_16[2] = {};
    aruwlib::arch::convertToBigEndian((uint16_t)0x0011, data_output_16);
    ASSERT_EQ(data_output_16[0], 0x00);
    ASSERT_EQ(data_output_16[1], 0x11);

    // 32 bit
    uint8_t data_output_32[4] = {};
    aruwlib::arch::convertToBigEndian((uint32_t)0x00112233, data_output_32);
    ASSERT_EQ(data_output_32[0], 0x00);
    ASSERT_EQ(data_output_32[1], 0x11);
    ASSERT_EQ(data_output_32[2], 0x22);
    ASSERT_EQ(data_output_32[3], 0x33);

    // 64 bit
    uint8_t data_output_64[8] = {};
    aruwlib::arch::convertToBigEndian((uint64_t)0x0011223344556677, data_output_64);
    ASSERT_EQ(data_output_64[0], 0x00);
    ASSERT_EQ(data_output_64[1], 0x11);
    ASSERT_EQ(data_output_64[2], 0x22);
    ASSERT_EQ(data_output_64[3], 0x33);
    ASSERT_EQ(data_output_64[4], 0x44);
    ASSERT_EQ(data_output_64[5], 0x55);
    ASSERT_EQ(data_output_64[6], 0x66);
    ASSERT_EQ(data_output_64[7], 0x77);
}

TEST(EndiannessWrappersTest, FromLittleEndian)
{
    // 16 bit
    uint16_t data_16 = 0;
    uint8_t data_input_16[2] = {0x11, 0x00};
    aruwlib::arch::convertFromLittleEndian(&data_16, data_input_16);
    ASSERT_EQ(data_16, 0x0011);

    // 32 bit
    uint32_t data_32;
    uint8_t data_input_32[4] = {0x33, 0x22, 0x11, 0x00};
    aruwlib::arch::convertFromLittleEndian(&data_32, data_input_32);
    ASSERT_EQ(data_32, 0x00112233);

    // 64 bit
    uint64_t data_64;
    uint8_t data_input_64[8] = {0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00};
    aruwlib::arch::convertFromLittleEndian(&data_64, data_input_64);
    ASSERT_EQ(data_64, 0x0011223344556677);
}

TEST(EndiannessWrappersTest, FromBigEndian)
{
    // 16 bit
    uint16_t data_16;
    uint8_t data_input_16[2] = {0x00, 0x11};
    aruwlib::arch::convertFromBigEndian(&data_16, data_input_16);
    ASSERT_EQ(data_16, 0x0011);

    // 32 bit
    uint32_t data_32;
    uint8_t data_input_32[4] = {0x00, 0x11, 0x22, 0x33};
    aruwlib::arch::convertFromBigEndian(&data_32, data_input_32);
    ASSERT_EQ(data_32, 0x00112233);

    // 64 bit
    uint64_t data_64;
    uint8_t data_input_64[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
    aruwlib::arch::convertFromBigEndian(&data_64, data_input_64);
    ASSERT_EQ(data_64, 0x0011223344556677);
}