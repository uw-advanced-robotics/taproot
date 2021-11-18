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

#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/drivers.hpp"

using namespace tap;
using namespace tap::serial;
using namespace tap::arch;

TEST(RefSerial, messageReceiveCallback__competition_status)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 11;
    msg.sequenceNumber = 0;
    msg.type = 1;
    memset(msg.data, 0, sizeof(msg.data));
    msg.data[0] = 2 << 4;
    convertToLittleEndian<uint16_t>(200, msg.data + 1);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(200, refSerial.getGameData().stageTimeRemaining);
    EXPECT_EQ(RefSerial::Rx::GameStage::INITIALIZATION, refSerial.getGameData().gameStage);

    msg.data[0] = 4 << 4;

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameStage::IN_GAME, refSerial.getGameData().gameStage);
}

TEST(RefSerial, messageReceiveCallback__competition_result)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 1;
    msg.sequenceNumber = 0;
    msg.type = 2;
    msg.data[0] = 0;

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameWinner::DRAW, refSerial.getGameData().gameWinner);

    msg.data[0] = 1;
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameWinner::RED, refSerial.getGameData().gameWinner);

    msg.data[0] = 2;
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameWinner::BLUE, refSerial.getGameData().gameWinner);
}

TEST(RefSerial, messageReceiveCallback__robot_hp)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 32;
    msg.sequenceNumber = 0;
    msg.type = 3;
    convertToLittleEndian<uint16_t>(1, msg.data);
    convertToLittleEndian<uint16_t>(2, msg.data + 2);
    convertToLittleEndian<uint16_t>(3, msg.data + 4);
    convertToLittleEndian<uint16_t>(4, msg.data + 6);
    convertToLittleEndian<uint16_t>(5, msg.data + 8);
    convertToLittleEndian<uint16_t>(6, msg.data + 10);
    convertToLittleEndian<uint16_t>(7, msg.data + 12);
    convertToLittleEndian<uint16_t>(8, msg.data + 14);
    convertToLittleEndian<uint16_t>(9, msg.data + 16);
    convertToLittleEndian<uint16_t>(10, msg.data + 18);
    convertToLittleEndian<uint16_t>(11, msg.data + 20);
    convertToLittleEndian<uint16_t>(12, msg.data + 22);
    convertToLittleEndian<uint16_t>(13, msg.data + 24);
    convertToLittleEndian<uint16_t>(14, msg.data + 26);
    convertToLittleEndian<uint16_t>(15, msg.data + 28);
    convertToLittleEndian<uint16_t>(16, msg.data + 30);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(1, refSerial.getRobotData().allRobotHp.red.hero1);
    EXPECT_EQ(2, refSerial.getRobotData().allRobotHp.red.engineer2);
    EXPECT_EQ(3, refSerial.getRobotData().allRobotHp.red.standard3);
    EXPECT_EQ(4, refSerial.getRobotData().allRobotHp.red.standard4);
    EXPECT_EQ(5, refSerial.getRobotData().allRobotHp.red.standard5);
    EXPECT_EQ(6, refSerial.getRobotData().allRobotHp.red.sentry7);
    EXPECT_EQ(7, refSerial.getRobotData().allRobotHp.red.outpost);
    EXPECT_EQ(8, refSerial.getRobotData().allRobotHp.red.base);
    EXPECT_EQ(9, refSerial.getRobotData().allRobotHp.blue.hero1);
    EXPECT_EQ(10, refSerial.getRobotData().allRobotHp.blue.engineer2);
    EXPECT_EQ(11, refSerial.getRobotData().allRobotHp.blue.standard3);
    EXPECT_EQ(12, refSerial.getRobotData().allRobotHp.blue.standard4);
    EXPECT_EQ(13, refSerial.getRobotData().allRobotHp.blue.standard5);
    EXPECT_EQ(14, refSerial.getRobotData().allRobotHp.blue.sentry7);
    EXPECT_EQ(15, refSerial.getRobotData().allRobotHp.blue.outpost);
    EXPECT_EQ(16, refSerial.getRobotData().allRobotHp.blue.base);
}

TEST(RefSerial, messageReceiveCallback__robot_status)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 27;
    memset(msg.data, 0, sizeof(msg.data));
    msg.sequenceNumber = 0;
    msg.type = 0x201;

    // red team
    for (int i = static_cast<int>(RefSerial::RobotId::RED_HERO);
         i <= static_cast<int>(RefSerial::RobotId::RED_RADAR_STATION);
         i++)
    {
        msg.data[0] = i;

        refSerial.messageReceiveCallback(msg);

        EXPECT_EQ(i, static_cast<int>(refSerial.getRobotData().robotId));
    }

    // blue team
    for (int i = static_cast<int>(RefSerial::RobotId::BLUE_HERO);
         i <= static_cast<int>(RefSerial::RobotId::BLUE_RADAR_STATION);
         i++)
    {
        msg.data[0] = i;

        refSerial.messageReceiveCallback(msg);

        EXPECT_EQ(i, static_cast<int>(refSerial.getRobotData().robotId));
    }

    msg.data[1] = 3;                                      // Robot level
    convertToLittleEndian<uint16_t>(1001, msg.data + 2);  // The robot’s remaining HP
    convertToLittleEndian<uint16_t>(2001, msg.data + 4);  // Robot maximum HP
    convertToLittleEndian<uint16_t>(120,
                                    msg.data + 6);        // Robot 1 17mm barrel cooling value
    convertToLittleEndian<uint16_t>(100, msg.data + 8);   // Robot 1 17mm barrel heat limit
    convertToLittleEndian<uint16_t>(240, msg.data + 10);  // Robot 1 17mm barrel speed limit (m/s)
    convertToLittleEndian<uint16_t>(4234,
                                    msg.data + 12);      // Robot 2 17mm barrel cooling value
    convertToLittleEndian<uint16_t>(12, msg.data + 14);  // Robot 2 17mm barrel heat limit
    convertToLittleEndian<uint16_t>(15, msg.data + 16);  // Robot 2 17mm barrel speed limit (m/s)
    convertToLittleEndian<uint16_t>(400,
                                    msg.data + 18);       // Robot 42mm barrel cooling value
    convertToLittleEndian<uint16_t>(439, msg.data + 20);  // Robot 42mm barrel heat limit
    convertToLittleEndian<uint16_t>(13, msg.data + 22);   // Robot 42mm barrel speed limit (m/s)
    convertToLittleEndian<uint16_t>(987, msg.data + 24);  // Robot chassis power consumption limit

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(3, refSerial.getRobotData().robotLevel);
    EXPECT_EQ(1001, refSerial.getRobotData().currentHp);
    EXPECT_EQ(2001, refSerial.getRobotData().maxHp);
    EXPECT_EQ(120, refSerial.getRobotData().turret.heatCoolingRate17ID1);
    EXPECT_EQ(100, refSerial.getRobotData().turret.heatLimit17ID1);
    EXPECT_EQ(240, refSerial.getRobotData().turret.barrelSpeedLimit17ID1);
    EXPECT_EQ(4234, refSerial.getRobotData().turret.heatCoolingRate17ID2);
    EXPECT_EQ(12, refSerial.getRobotData().turret.heatLimit17ID2);
    EXPECT_EQ(15, refSerial.getRobotData().turret.barrelSpeedLimit17ID2);
    EXPECT_EQ(400, refSerial.getRobotData().turret.heatCoolingRate42);
    EXPECT_EQ(439, refSerial.getRobotData().turret.heatLimit42);
    EXPECT_EQ(13, refSerial.getRobotData().turret.barrelSpeedLimit42);
    EXPECT_EQ(987, refSerial.getRobotData().chassis.powerConsumptionLimit);

    // Main controller power supply condition
    msg.data[26] = 0b001;

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER, refSerial.getRobotData().robotPower);

    msg.data[26] = 0b010;

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER, refSerial.getRobotData().robotPower);

    msg.data[26] = 0b100;

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::RobotPower::SHOOTER_HAS_POWER, refSerial.getRobotData().robotPower);

    msg.data[26] = 0b111;

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER | RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER |
            RefSerial::Rx::RobotPower::SHOOTER_HAS_POWER,
        refSerial.getRobotData().robotPower);
}

TEST(RefSerial, messageReceiveCallback__power_and_heat)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 16;
    msg.sequenceNumber = 0;
    msg.type = 0x202;
    memset(msg.data, 0, sizeof(msg.data));

    convertToLittleEndian<uint16_t>(1234, msg.data);      // Chassis output voltage (mV)
    convertToLittleEndian<uint16_t>(4321, msg.data + 2);  // Chassis output current (mA)
    convertToLittleEndian<float>(6789, msg.data + 4);     // Chassis output power (W)
    convertToLittleEndian<uint16_t>(120, msg.data + 8);   // Chassis power buffer (J)
    convertToLittleEndian<uint16_t>(145, msg.data + 10);  // No. 1 17mm barrel heat
    convertToLittleEndian<uint16_t>(431, msg.data + 12);  // No. 2 17mm barrel heat
    convertToLittleEndian<uint16_t>(900, msg.data + 14);  // 42mm barrel heat

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(1234, refSerial.getRobotData().chassis.volt);
    EXPECT_EQ(4321, refSerial.getRobotData().chassis.current);
    EXPECT_EQ(6789, refSerial.getRobotData().chassis.power);
    EXPECT_EQ(120, refSerial.getRobotData().chassis.powerBuffer);
    EXPECT_EQ(145, refSerial.getRobotData().turret.heat17ID1);
    EXPECT_EQ(431, refSerial.getRobotData().turret.heat17ID2);
    EXPECT_EQ(900, refSerial.getRobotData().turret.heat42);
}

TEST(RefSerial, messageReceiveCallback__robot_status_dps_measured)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 27;
    memset(msg.data, 0, sizeof(msg.data));
    msg.sequenceNumber = 0;
    msg.type = 0x201;

    // vector of arrays of length 3 containing a time, current HP, and expected DPS
    std::vector<std::array<int, 3>> timeHPExpectedDPS{
        {0, 100, 0},
        {10, 90, 10},
        {20, 90, 10},
        {30, 90, 10},
        {40, 80, 20},
        {1500, 80, 0}};

    for (const auto &triplet : timeHPExpectedDPS)
    {
        tap::arch::clock::setTime(triplet[0]);
        convertToLittleEndian<uint16_t>(triplet[1], msg.data + 2);

        refSerial.messageReceiveCallback(msg);

        EXPECT_EQ(triplet[1], refSerial.getRobotData().currentHp);
        EXPECT_EQ(triplet[2], refSerial.getRobotData().receivedDps);
    }
}

TEST(RefSerial, messageReceiveCallback__robot_position)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 16;
    msg.sequenceNumber = 0;
    msg.type = 0x203;
    memset(msg.data, 0, sizeof(msg.data));

    convertToLittleEndian<float>(-89.31f, msg.data);
    convertToLittleEndian<float>(45069.24f, msg.data + 4);
    convertToLittleEndian<float>(90.12f, msg.data + 8);
    convertToLittleEndian<float>(-799.87f, msg.data + 12);

    refSerial.messageReceiveCallback(msg);

    EXPECT_NEAR(-89.31f, refSerial.getRobotData().chassis.x, 1E-3f);
    EXPECT_NEAR(45069.24f, refSerial.getRobotData().chassis.y, 1E-3f);
    EXPECT_NEAR(90.12f, refSerial.getRobotData().chassis.z, 1E-3f);
    EXPECT_NEAR(-799.87f, refSerial.getRobotData().turret.yaw, 1E-3f);
}

TEST(RefSerial, messageReceiveCallback__robot_buffs)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 1;
    msg.sequenceNumber = 0;
    msg.type = 0x204;
    memset(msg.data, 0, sizeof(msg.data));

    msg.data[0] = 0b0001;
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotBuffStatus::ROBOT_HP_RESTORATION_STATUS,
        refSerial.getRobotData().robotBuffStatus);

    msg.data[0] = 0b0010;
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotBuffStatus::BARREL_HEAT_COOLING_ACCELERATION,
        refSerial.getRobotData().robotBuffStatus);

    msg.data[0] = 0b0100;
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotBuffStatus::ROBOT_DEFENSE_BUFF,
        refSerial.getRobotData().robotBuffStatus);

    msg.data[0] = 0b1000;
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotBuffStatus::ROBOT_ATTACK_BUFF,
        refSerial.getRobotData().robotBuffStatus);
}

TEST(RefSerial, messageReceiveCallback__aerial_robot_energy_status)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 2;
    msg.sequenceNumber = 0;
    msg.type = 0x205;
    memset(msg.data, 0, sizeof(msg.data));

    convertToLittleEndian(12345, msg.data);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(12345, refSerial.getRobotData().aerialEnergyStatus);
}

TEST(RefSerial, messageReceiveCallback__damage_status)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 1;
    msg.sequenceNumber = 0;
    msg.type = 0x206;
    memset(msg.data, 0, sizeof(msg.data));

    msg.data[0] = 0;

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::ArmorId::FRONT, refSerial.getRobotData().damagedArmorId);
    EXPECT_EQ(RefSerial::Rx::DamageType::ARMOR_DAMAGE, refSerial.getRobotData().damageType);

    msg.data[0] = (0x2) | ((0x1) << 4);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::ArmorId::REAR, refSerial.getRobotData().damagedArmorId);
    EXPECT_EQ(RefSerial::Rx::DamageType::MODULE_OFFLINE, refSerial.getRobotData().damageType);

    msg.data[0] = (0x4) | ((0x5) << 4);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::ArmorId::TOP, refSerial.getRobotData().damagedArmorId);
    EXPECT_EQ(RefSerial::Rx::DamageType::COLLISION, refSerial.getRobotData().damageType);
}

TEST(RefSerial, messageReceiveCallback__launching_information)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 6;
    msg.sequenceNumber = 0;
    msg.type = 0x207;
    memset(msg.data, 0, sizeof(msg.data));

    refSerial.messageReceiveCallback(msg);
}

TEST(RefSerial, messageReceiveCallback__remaining_projectiles)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 6;
    msg.sequenceNumber = 0;
    msg.type = 0x208;
    memset(msg.data, 0, sizeof(msg.data));

    convertToLittleEndian<uint16_t>(123, msg.data);
    convertToLittleEndian<uint16_t>(1890, msg.data + 2);
    convertToLittleEndian<uint16_t>(12892, msg.data + 4);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(123, refSerial.getRobotData().turret.bulletsRemaining17);
    EXPECT_EQ(1890, refSerial.getRobotData().turret.bulletsRemaining42);
    EXPECT_EQ(12892, refSerial.getRobotData().remainingCoins);
}

TEST(RefSerial, messageReceiveCallback__RFID_status)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 4;
    msg.sequenceNumber = 0;
    msg.type = 0x209;
    memset(msg.data, 0, sizeof(msg.data));

    msg.data[0] = 0b00000101;

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(msg.data[0], refSerial.getRobotData().rfidStatus.value);

    msg.data[0] = 0b11111010;

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(msg.data[0], refSerial.getRobotData().rfidStatus.value);
}

TEST(RefSerial, messageReceiveCallback__interaction_data_simple_message)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    DJISerial::SerialMessage msg;
    msg.length = 0;
    msg.sequenceNumber = 0;
    msg.type = 0x301;
    memset(msg.data, 0, sizeof(msg.data));

    msg.data[0] = 'h';
    msg.data[1] = 'i';

    // refSerial.attachInteractionListener()

    refSerial.messageReceiveCallback(msg);

    // make sure listener is called
}
