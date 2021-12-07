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
#include "tap/mock/robot_to_robot_message_handler_mock.hpp"

using namespace tap;
using namespace tap::serial;
using namespace tap::arch;

template <typename T>
static DJISerial::SerialMessage constructMsg(const T &data, int type)
{
    DJISerial::SerialMessage msg;
    msg.sequenceNumber = 0;
    msg.type = type;
    msg.length = sizeof(T);
    memset(msg.data, 0, sizeof(msg.data));
    memcpy(msg.data, reinterpret_cast<const uint8_t *>(&data), sizeof(T));
    return msg;
}

TEST(RefSerial, messageReceiveCallback__competition_status)
{
    struct GameStatus
    {
        uint8_t gameType : 4;
        uint8_t gameProgress : 4;
        uint16_t stageRemainTime;
        uint64_t syncTimeStamp;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    GameStatus testData;

    testData.gameProgress = 2;
    testData.stageRemainTime = 200;
    msg = constructMsg(testData, 1);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(200, refSerial.getGameData().stageTimeRemaining);
    EXPECT_EQ(RefSerial::Rx::GameStage::INITIALIZATION, refSerial.getGameData().gameStage);

    testData.gameProgress = 4;
    msg = constructMsg(testData, 1);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameStage::IN_GAME, refSerial.getGameData().gameStage);
}

TEST(RefSerial, messageReceiveCallback__competition_result)
{
    struct GameResult
    {
        uint8_t winner;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    GameResult testData;

    testData.winner = 0;
    msg = constructMsg(testData, 2);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameWinner::DRAW, refSerial.getGameData().gameWinner);

    testData.winner = 1;
    msg = constructMsg(testData, 2);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameWinner::RED, refSerial.getGameData().gameWinner);

    testData.winner = 2;
    msg = constructMsg(testData, 2);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameWinner::BLUE, refSerial.getGameData().gameWinner);
}

TEST(RefSerial, messageReceiveCallback__robot_hp)
{
    struct GameRobotHP
    {
        uint16_t red1RobotHP;
        uint16_t red2RobotHP;
        uint16_t red3RobotHP;
        uint16_t red4RobotHP;
        uint16_t red5RobotHP;
        uint16_t red7RobotHP;
        uint16_t redOutpostHP;
        uint16_t redBaseHP;
        uint16_t blue1RobotHP;
        uint16_t blue2RobotHP;
        uint16_t blue3RobotHP;
        uint16_t blue4RobotHP;
        uint16_t blue5RobotHP;
        uint16_t blue7RobotHP;
        uint16_t blueOutpostHP;
        uint16_t blueBaseHP;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;

    GameRobotHP testData;

    testData.red1RobotHP = 1;
    testData.red2RobotHP = 2;
    testData.red3RobotHP = 3;
    testData.red4RobotHP = 4;
    testData.red5RobotHP = 5;
    testData.red7RobotHP = 6;
    testData.redOutpostHP = 7;
    testData.redBaseHP = 8;
    testData.blue1RobotHP = 9;
    testData.blue2RobotHP = 10;
    testData.blue3RobotHP = 11;
    testData.blue4RobotHP = 12;
    testData.blue5RobotHP = 13;
    testData.blue7RobotHP = 14;
    testData.blueOutpostHP = 15;
    testData.blueBaseHP = 16;
    msg = constructMsg(testData, 3);

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

struct GameRobotStatus
{
    uint8_t robotId;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooterId117mmCoolingRate;
    uint16_t shooterId117mmCoolingLimit;
    uint16_t shooterId117mmSpeedLimit;
    uint16_t shooterId217mmCoolingRate;
    uint16_t shooterId217mmCoolingLimit;
    uint16_t shooterId217mmSpeedLimit;
    uint16_t shooterId142mmCoolingLate;
    uint16_t shooterId142mmCoolingLimit;
    uint16_t shooterId142mmSpeedLimit;
    uint16_t chassisPowerLimit;
    uint8_t mainsPowerGimbalOutput : 1;
    uint8_t mainsPowerChassisOutput : 1;
    uint8_t mainsPowerShooterOutput : 1;
} modm_packed;

TEST(RefSerial, messageReceiveCallback__robot_status)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    GameRobotStatus testData;

    testData.robot_level = 3;
    testData.remain_HP = 1001;
    testData.max_HP = 2001;
    testData.shooterId117mmCoolingRate = 120;
    testData.shooterId117mmCoolingLimit = 100;
    testData.shooterId117mmSpeedLimit = 240;
    testData.shooterId217mmCoolingRate = 4234;
    testData.shooterId217mmCoolingLimit = 12;
    testData.shooterId217mmSpeedLimit = 15;
    testData.shooterId142mmCoolingLate = 400;
    testData.shooterId142mmCoolingLimit = 439;
    testData.shooterId142mmSpeedLimit = 13;
    testData.chassisPowerLimit = 987;
    msg = constructMsg(testData, 0x0201);

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

    // red team
    for (int i = static_cast<int>(RefSerial::RobotId::RED_HERO);
         i <= static_cast<int>(RefSerial::RobotId::RED_RADAR_STATION);
         i++)
    {
        testData.robotId = i;
        msg = constructMsg(testData, 0x0201);

        refSerial.messageReceiveCallback(msg);

        EXPECT_EQ(i, static_cast<int>(refSerial.getRobotData().robotId));
    }

    // blue team
    for (int i = static_cast<int>(RefSerial::RobotId::BLUE_HERO);
         i <= static_cast<int>(RefSerial::RobotId::BLUE_RADAR_STATION);
         i++)
    {
        testData.robotId = i;
        msg = constructMsg(testData, 0x0201);

        refSerial.messageReceiveCallback(msg);

        EXPECT_EQ(i, static_cast<int>(refSerial.getRobotData().robotId));
    }

    // Main controller power supply condition

    testData.mainsPowerGimbalOutput = 1;
    testData.mainsPowerChassisOutput = 0;
    testData.mainsPowerShooterOutput = 0;
    msg = constructMsg(testData, 0x0201);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER, refSerial.getRobotData().robotPower);

    testData.mainsPowerGimbalOutput = 0;
    testData.mainsPowerChassisOutput = 1;
    testData.mainsPowerShooterOutput = 0;
    msg = constructMsg(testData, 0x0201);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER, refSerial.getRobotData().robotPower);

    testData.mainsPowerGimbalOutput = 0;
    testData.mainsPowerChassisOutput = 0;
    testData.mainsPowerShooterOutput = 1;
    msg = constructMsg(testData, 0x0201);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::RobotPower::SHOOTER_HAS_POWER, refSerial.getRobotData().robotPower);

    testData.mainsPowerGimbalOutput = 1;
    testData.mainsPowerChassisOutput = 1;
    testData.mainsPowerShooterOutput = 1;
    msg = constructMsg(testData, 0x0201);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER | RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER |
            RefSerial::Rx::RobotPower::SHOOTER_HAS_POWER,
        refSerial.getRobotData().robotPower);
}

TEST(RefSerial, messageReceiveCallback__robot_status_dps_measured)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    GameRobotStatus testData;

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

        testData.remain_HP = triplet[1];
        msg = constructMsg(testData, 0x0201);
        refSerial.messageReceiveCallback(msg);

        EXPECT_EQ(triplet[1], refSerial.getRobotData().currentHp);
        EXPECT_EQ(triplet[2], refSerial.getRobotData().receivedDps);
    }
}

TEST(RefSerial, messageReceiveCallback__power_and_heat)
{
    struct PowerHeatData
    {
        uint16_t chassisVolt;
        uint16_t chassisCurrent;
        float chassisPower;
        uint16_t chassis_power_buffer;
        uint16_t shooterId117mmCoolingHeat;
        uint16_t shooterId217mmCoolingHeat;
        uint16_t shooterId142mmCoolingHeat;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    PowerHeatData testData;

    testData.chassisVolt = 1234;
    testData.chassisCurrent = 4321;
    testData.chassisPower = 6789;
    testData.chassis_power_buffer = 120;
    testData.shooterId117mmCoolingHeat = 145;
    testData.shooterId217mmCoolingHeat = 431;
    testData.shooterId142mmCoolingHeat = 900;
    msg = constructMsg(testData, 0x0202);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(1234, refSerial.getRobotData().chassis.volt);
    EXPECT_EQ(4321, refSerial.getRobotData().chassis.current);
    EXPECT_EQ(6789, refSerial.getRobotData().chassis.power);
    EXPECT_EQ(120, refSerial.getRobotData().chassis.powerBuffer);
    EXPECT_EQ(145, refSerial.getRobotData().turret.heat17ID1);
    EXPECT_EQ(431, refSerial.getRobotData().turret.heat17ID2);
    EXPECT_EQ(900, refSerial.getRobotData().turret.heat42);
}

TEST(RefSerial, messageReceiveCallback__robot_position)
{
    struct RobotPosData
    {
        float x;
        float y;
        float z;
        float yaw;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    RobotPosData testData;

    testData.x = -89.31f;
    testData.y = 45069.24f;
    testData.z = 90.12f;
    testData.yaw = -799.87f;
    msg = constructMsg(testData, 0x0203);

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

    msg = constructMsg(static_cast<uint8_t>(0b0001), 0x0204);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotBuffStatus::ROBOT_HP_RESTORATION_STATUS,
        refSerial.getRobotData().robotBuffStatus);

    msg = constructMsg(static_cast<uint8_t>(0b0010), 0x0204);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotBuffStatus::BARREL_HEAT_COOLING_ACCELERATION,
        refSerial.getRobotData().robotBuffStatus);

    msg = constructMsg(static_cast<uint8_t>(0b0100), 0x0204);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotBuffStatus::ROBOT_DEFENSE_BUFF,
        refSerial.getRobotData().robotBuffStatus);

    msg = constructMsg(static_cast<uint8_t>(0b1000), 0x0204);
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
    msg = constructMsg(static_cast<uint16_t>(12345), 0x0205);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(12345, refSerial.getRobotData().aerialEnergyStatus);
}

TEST(RefSerial, messageReceiveCallback__damage_status)
{
    struct RobotDamage
    {
        uint8_t armorId : 4;
        uint8_t hurtType : 4;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    RobotDamage testData;

    testData.armorId = 0;
    testData.hurtType = 0;
    msg = constructMsg(testData, 0x0206);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::ArmorId::FRONT, refSerial.getRobotData().damagedArmorId);
    EXPECT_EQ(RefSerial::Rx::DamageType::ARMOR_DAMAGE, refSerial.getRobotData().damageType);

    testData.armorId = 2;
    testData.hurtType = 1;
    msg = constructMsg(testData, 0x0206);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::ArmorId::REAR, refSerial.getRobotData().damagedArmorId);
    EXPECT_EQ(RefSerial::Rx::DamageType::MODULE_OFFLINE, refSerial.getRobotData().damageType);

    testData.armorId = 4;
    testData.hurtType = 5;
    msg = constructMsg(testData, 0x0206);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::ArmorId::TOP, refSerial.getRobotData().damagedArmorId);
    EXPECT_EQ(RefSerial::Rx::DamageType::COLLISION, refSerial.getRobotData().damageType);
}

TEST(RefSerial, messageReceiveCallback__launching_information)
{
    struct ShootData
    {
        uint8_t bulletType;
        uint8_t shooterId;
        uint8_t bulletFreq;
        float bulletSpeed;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    ShootData testData;

    testData.bulletType = 1;
    testData.shooterId = 1;
    testData.bulletFreq = 45;
    testData.bulletSpeed = 3452.12f;
    msg = constructMsg(testData, 0x0207);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::BulletType::AMMO_17, refSerial.getRobotData().turret.bulletType);
    EXPECT_EQ(
        RefSerial::Rx::MechanismID::TURRET_17MM_1,
        refSerial.getRobotData().turret.launchMechanismID);
    EXPECT_EQ(45, refSerial.getRobotData().turret.firingFreq);
    EXPECT_NEAR(3452.12f, refSerial.getRobotData().turret.bulletSpeed, 1E-3);

    testData.bulletType = 2;
    testData.shooterId = 2;
    msg = constructMsg(testData, 0x0207);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::BulletType::AMMO_42, refSerial.getRobotData().turret.bulletType);
    EXPECT_EQ(
        RefSerial::Rx::MechanismID::TURRET_17MM_2,
        refSerial.getRobotData().turret.launchMechanismID);
}

TEST(RefSerial, messageReceiveCallback__remaining_projectiles)
{
    struct BulletRemaining
    {
        uint16_t bulletRemainingNum17mm;
        uint16_t bulletRemainingNum42mm;
        uint16_t coinRemainingNum;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    BulletRemaining testData;

    testData.bulletRemainingNum17mm = 123;
    testData.bulletRemainingNum42mm = 1890;
    testData.coinRemainingNum = 12892;
    msg = constructMsg(testData, 0x0208);

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

    msg = constructMsg(static_cast<uint32_t>(0b00000101), 0x0209);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(msg.data[0], refSerial.getRobotData().rfidStatus.value);

    msg = constructMsg(static_cast<uint32_t>(0b11111010), 0x0209);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(msg.data[0], refSerial.getRobotData().rfidStatus.value);
}

TEST(RefSerial, configGraphicGenerics__sets_name_operation_layer_color)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    RefSerial::Tx::GraphicData data;

    uint8_t name[3] = {0, 1, 0};

    refSerial.configGraphicGenerics(
        &data,
        name,
        RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC_MODIFY,
        0,
        RefSerial::Tx::GraphicColor::PINK);

    EXPECT_TRUE(0 == std::memcmp(name, data.name, sizeof(name)));
    EXPECT_EQ(RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC_MODIFY, data.operation);
    EXPECT_EQ(static_cast<uint8_t>(RefSerial::Tx::GraphicColor::PINK), data.color);
}

static void updateRobotId(RefSerial &refSerial, RefSerial::RobotId id)
{
    DJISerial::SerialMessage msg;
    GameRobotStatus testData;
    testData.remain_HP = 300;
    testData.robotId = static_cast<uint16_t>(id);
    msg = constructMsg(testData, 0x0201);
    refSerial.messageReceiveCallback(msg);
}

TEST(RefSerial, deleteGraphicLayer__doesnt_send_if_robot_id_invalid)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    RefSerial::Tx::DeleteGraphicOperation op =
        RefSerial::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER;

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerial.deleteGraphicLayer(op, 1);
}

TEST(RefSerial, deleteGraphicLayer__sends_correct_msg)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    updateRobotId(refSerial, RefSerial::RobotId::RED_SOLDIER_3);

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_))
        .WillOnce([&](tap::serial::Uart::UartPort, const uint8_t *data, std::size_t length) {
            const RefSerial::Tx::DeleteGraphicLayerMessage *msg =
                reinterpret_cast<const RefSerial::Tx::DeleteGraphicLayerMessage *>(data);

            EXPECT_EQ(
                sizeof(msg->interactiveHeader) + sizeof(msg->deleteOperation) + sizeof(msg->layer),
                msg->frameHeader.dataLength);
            uint16_t cmdId = *reinterpret_cast<const uint16_t *>(data + sizeof(msg->frameHeader));
            EXPECT_EQ(0x0301, cmdId);
            EXPECT_EQ(0xa5, msg->frameHeader.SOF);
            EXPECT_EQ(
                tap::algorithms::calculateCRC8(data, sizeof(msg->frameHeader) - 1),
                msg->frameHeader.CRC8);

            EXPECT_EQ(0x0100, msg->interactiveHeader.dataCmdId);
            EXPECT_EQ(
                0x0100 + static_cast<uint16_t>(RefSerial::RobotId::RED_SOLDIER_3),
                msg->interactiveHeader.receiverId);
            EXPECT_EQ(
                static_cast<uint16_t>(RefSerial::RobotId::RED_SOLDIER_3),
                msg->interactiveHeader.senderId);

            EXPECT_EQ(0x0301, msg->cmdId);
            EXPECT_EQ(
                static_cast<uint16_t>(RefSerial::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER),
                msg->deleteOperation);
            EXPECT_EQ(2, msg->layer);

            return length;
        });

    refSerial.deleteGraphicLayer(RefSerial::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER, 2);
}

TEST(RefSerial, sendGraphic__1_doesnt_send_if_robot_id_invalid)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    RefSerial::Tx::Graphic1Message msg{};

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerial.sendGraphic(&msg);
}

TEST(RefSerial, sendGraphic__1_doesnt_sen_but_configures_if_sendMsg_false)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    RefSerial::Tx::Graphic1Message msg{};

    updateRobotId(refSerial, RefSerial::RobotId::BLUE_SOLDIER_1);

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerial.sendGraphic(&msg, true, false);

    // validate the msg header was still constructed
    EXPECT_EQ(0x0301, msg.cmdId);
    EXPECT_EQ(sizeof(msg.graphicData) + sizeof(msg.interactiveHeader), msg.frameHeader.dataLength);
    EXPECT_EQ(0xa5, msg.frameHeader.SOF);
    EXPECT_EQ(
        static_cast<uint16_t>(RefSerial::RobotId::BLUE_SOLDIER_1),
        msg.interactiveHeader.senderId);
    EXPECT_EQ(
        0x100 + static_cast<uint16_t>(RefSerial::RobotId::BLUE_SOLDIER_1),
        msg.interactiveHeader.receiverId);
}

TEST(RefSerial, sendGraphic__1_doesnt_send_or_config_if_configMsgHeader_sendMsg_false)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    RefSerial::Tx::Graphic1Message msg{};
    msg.cmdId = 0x1;

    updateRobotId(refSerial, RefSerial::RobotId::BLUE_SOLDIER_1);

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerial.sendGraphic(&msg, false, false);

    // validate the msg header was still constructed
    EXPECT_EQ(0x1, msg.cmdId);
}

TEST(RefSerial, sendGraphic__characterMessage)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    RefSerial::Tx::GraphicCharacterMessage msg{};

    updateRobotId(refSerial, RefSerial::RobotId::BLUE_ENGINEER);

    msg.msg[0] = 'h';
    msg.msg[1] = 'e';
    msg.msg[2] = 'l';
    msg.msg[3] = 'l';
    msg.msg[4] = 'o';
    msg.msg[5] = '\0';

    EXPECT_CALL(
        drivers.uart,
        write(testing::_, testing::_, sizeof(RefSerial::Tx::GraphicCharacterMessage)))
        .WillOnce([&](tap::serial::Uart::UartPort, const uint8_t *data, std::size_t length) {
            const auto header = reinterpret_cast<const RefSerial::Tx::FrameHeader *>(data);
            EXPECT_EQ(
                sizeof(msg.interactiveHeader) + sizeof(msg.graphicData) + sizeof(msg.msg),
                header->dataLength);

            uint16_t cmdId = *reinterpret_cast<const uint16_t *>(data + sizeof(msg.frameHeader));
            EXPECT_EQ(0x0301, cmdId);

            const auto interactiveHeader =
                reinterpret_cast<const RefSerial::Tx::InteractiveHeader *>(
                    data + sizeof(msg.frameHeader) + sizeof(msg.cmdId));
            EXPECT_EQ(0x0110, interactiveHeader->dataCmdId);
            EXPECT_EQ(
                RefSerial::RobotId::BLUE_ENGINEER,
                static_cast<RefSerial::RobotId>(interactiveHeader->senderId));
            EXPECT_EQ(
                0x100 + static_cast<uint16_t>(RefSerial::RobotId::BLUE_ENGINEER),
                interactiveHeader->receiverId);

            // Don't care about graphic data, only msg

            const uint8_t *msgData = data + sizeof(msg.frameHeader) + sizeof(msg.cmdId) +
                                     sizeof(msg.interactiveHeader) + sizeof(msg.graphicData);
            for (size_t i = 0; i < 6; i++)
            {
                EXPECT_EQ(msg.msg[i], msgData[i]);
            }

            // Validate crc16
            EXPECT_EQ(
                tap::algorithms::calculateCRC16(
                    data,
                    sizeof(RefSerial::Tx::GraphicCharacterMessage) - sizeof(uint16_t)),
                *reinterpret_cast<const uint16_t *>(
                    data + sizeof(RefSerial::Tx::GraphicCharacterMessage) - sizeof(uint16_t)));

            return length;
        });

    refSerial.sendGraphic(&msg);
}

TEST(RefSerial, sendRobotToRobotMessage__invalid_id_fails_to_send)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    RefSerial::Tx::RobotToRobotMessage msg{};

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    refSerial.sendRobotToRobotMsg(&msg, 0x01ff, RefSerial::RobotId::RED_HERO, 2);
    refSerial.sendRobotToRobotMsg(&msg, 0x1000, RefSerial::RobotId::RED_HERO, 2);
}

TEST(RefSerial, sendRobotToRobotMessage__validate_sending_msg_to_same_color_robot_works)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    RefSerial::Tx::RobotToRobotMessage msg;

    updateRobotId(refSerial, RefSerial::RobotId::RED_DRONE);

    msg.dataAndCRC16[0] = 'h';
    msg.dataAndCRC16[1] = 'i';

    static constexpr int msgLen = 2;

    static constexpr int entireMsgLen = sizeof(msg.frameHeader) + sizeof(msg.cmdId) +
                                        sizeof(msg.interactiveHeader) + msgLen + sizeof(uint16_t);

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, entireMsgLen))
        .WillOnce([&](tap::serial::Uart::UartPort, const uint8_t *data, std::size_t length) {
            // Decode and validate header
            const RefSerial::Tx::FrameHeader *header =
                reinterpret_cast<const RefSerial::Tx::FrameHeader *>(data);
            EXPECT_EQ(sizeof(msg.interactiveHeader) + msgLen, header->dataLength);
            EXPECT_EQ(0xa5, header->SOF);
            EXPECT_EQ(
                tap::algorithms::calculateCRC8(data, sizeof(RefSerial::Tx::FrameHeader) - 1),
                header->CRC8);

            // Decode and validate interactive header
            const RefSerial::Tx::InteractiveHeader *interactiveHeader =
                reinterpret_cast<const RefSerial::Tx::InteractiveHeader *>(
                    data + sizeof(msg.frameHeader) + sizeof(msg.cmdId));
            EXPECT_EQ(0x0200, interactiveHeader->dataCmdId);
            EXPECT_EQ(
                RefSerial::RobotId::RED_HERO,
                static_cast<RefSerial::RobotId>(interactiveHeader->receiverId));
            EXPECT_EQ(
                RefSerial::RobotId::RED_DRONE,
                static_cast<RefSerial::RobotId>(interactiveHeader->senderId));

            // Decode and validate message
            static constexpr int START_DATA_OFFSET =
                sizeof(msg.frameHeader) + sizeof(msg.cmdId) + sizeof(msg.interactiveHeader);
            EXPECT_EQ('h', data[START_DATA_OFFSET]);
            EXPECT_EQ('i', data[START_DATA_OFFSET + 1]);

            // Validate crc16
            EXPECT_EQ(
                tap::algorithms::calculateCRC16(data, entireMsgLen - sizeof(uint16_t)),
                *reinterpret_cast<const uint16_t *>(data + entireMsgLen - sizeof(uint16_t)));
            return length;
        });

    refSerial.sendRobotToRobotMsg(&msg, 0x0200, RefSerial::RobotId::RED_HERO, 2);
}

TEST(
    RefSerial,
    getRobotIdBasedOnCurrentRobotTeam__returns_proper_robot_id_based_on_current_robot_id)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    EXPECT_EQ(
        RefSerial::RobotId::INVALID,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::INVALID));

    updateRobotId(refSerial, RefSerial::RobotId::BLUE_SENTINEL);

    EXPECT_EQ(
        RefSerial::RobotId::INVALID,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::INVALID));
    EXPECT_EQ(
        RefSerial::RobotId::BLUE_SOLDIER_2,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::BLUE_SOLDIER_2));
    EXPECT_EQ(
        RefSerial::RobotId::BLUE_SOLDIER_2,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::RED_SOLDIER_2));

    updateRobotId(refSerial, RefSerial::RobotId::RED_RADAR_STATION);

    EXPECT_EQ(
        RefSerial::RobotId::RED_HERO,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::BLUE_HERO));
    EXPECT_EQ(
        RefSerial::RobotId::RED_HERO,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::RED_HERO));
}

TEST(RefSerial, attachRobotToRobotMessageHandler__fails_to_add_if_msgId_out_of_bounsd)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    tap::mock::RobotToRobotMessageHandlerMock handler;

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    refSerial.attachRobotToRobotMessageHandler(0x014, &handler);
    refSerial.attachRobotToRobotMessageHandler(0x3ff, &handler);
}

TEST(RefSerial, attachRobotToRobotMessageHandler__fails_to_add_if_msgId_already_added)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    tap::mock::RobotToRobotMessageHandlerMock handler;

    refSerial.attachRobotToRobotMessageHandler(0x201, &handler);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(1);

    refSerial.attachRobotToRobotMessageHandler(0x201, &handler);
}

TEST(RefSerial, messageReceiveCallback__robot_to_robot_data_simple_message)
{
    struct SpecialData
    {
        RefSerial::Tx::InteractiveHeader interactiveHeader;
        uint8_t hi[2];
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::SerialMessage msg;
    SpecialData specialData;

    tap::mock::RobotToRobotMessageHandlerMock handler;

    specialData.interactiveHeader.dataCmdId = 0x201;
    specialData.hi[0] = 'h';
    specialData.hi[1] = 'i';
    msg = constructMsg(specialData, 0x301);

    refSerial.attachRobotToRobotMessageHandler(0x201, &handler);

    EXPECT_CALL(handler, functorOp).WillOnce([&](const DJISerial::SerialMessage &message) {
        EXPECT_EQ('h', message.data[sizeof(RefSerial::Tx::InteractiveHeader)]);
        EXPECT_EQ('i', message.data[sizeof(RefSerial::Tx::InteractiveHeader) + 1]);
        EXPECT_EQ(sizeof(SpecialData), message.length);
    });

    refSerial.messageReceiveCallback(msg);
}
