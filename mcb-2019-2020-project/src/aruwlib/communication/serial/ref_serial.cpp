#include "ref_serial.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"

namespace aruwlib
{

namespace serial
{

RefSerial RefSerial::refSerial;

RefSerial::RefSerial() :
DJISerial(Uart::UartPort::Uart6, true),
robotData(),
gameData(),
receivedDpsTracker()
{}

RefSerial& RefSerial::getRefSerial()
{
    return refSerial;
}

// rx stuff
void RefSerial::messageReceiveCallback(const SerialMessage& completeMessage)
{
    updateReceivedDamage();
    switch(completeMessage.type)
    {
        case REF_MESSAGE_TYPE_GAME_STATUS:
        {
            decodeToGameStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_GAME_RESULT:
        {
            decodeToGameResult(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ALL_ROBOT_HP:
        {
            decodeToAllRobotHP(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_STATUS:
        {
            decodeToRobotStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_POWER_AND_HEAT:
        {
            decodeToPowerAndHeat(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_POSITION:
        {
            decodeToRobotPosition(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_RECEIVE_DAMAGE:
        {
            decodeToReceiveDamage(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_PROJECTILE_LAUNCH:
        {
            decodeToProjectileLaunch(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_SENTINEL_DRONE_BULLETS_REMAIN:
        {
            decodeToSentinelDroneBulletsRemain(completeMessage);
            break;
        }
        default :
            break;
    }
}

// tx stuff
void RefSerial::sendDisplayData(const DisplayData& displayData)
{
    CustomData customData;
    customData.type = REF_CUSTOM_DATA_TYPE_UI_INDICATOR;
    customData.senderId = getRobotClientID(robotData.robotId);

    // 3 float variables to display on the referee client UI
    const uint32_t ref_comms_float_to_display1
        = aruwlib::algorithms::reinterpretCopy<float, uint32_t>(displayData.float1);
    const uint32_t ref_comms_float_to_display2
        = aruwlib::algorithms::reinterpretCopy<float, uint32_t>(displayData.float2);
    const uint32_t ref_comms_float_to_display3
        = aruwlib::algorithms::reinterpretCopy<float, uint32_t>(displayData.float3);

    // 3 custom floats to display
    uint8_t data[13] = {
        static_cast<uint8_t>(ref_comms_float_to_display1),
        static_cast<uint8_t>(ref_comms_float_to_display1 >> 8),
        static_cast<uint8_t>(ref_comms_float_to_display1 >> 16),
        static_cast<uint8_t>(ref_comms_float_to_display1 >> 24),

        static_cast<uint8_t>(ref_comms_float_to_display2),
        static_cast<uint8_t>(ref_comms_float_to_display2 >> 8),
        static_cast<uint8_t>(ref_comms_float_to_display2 >> 16),
        static_cast<uint8_t>(ref_comms_float_to_display2 >> 24),

        static_cast<uint8_t>(ref_comms_float_to_display3),
        static_cast<uint8_t>(ref_comms_float_to_display3 >> 8),
        static_cast<uint8_t>(ref_comms_float_to_display3 >> 16),
        static_cast<uint8_t>(ref_comms_float_to_display3 >> 24),

        // 6 custom boolean indicators to display in a single 8-bit value
        static_cast<uint8_t>(packBoolMask(
            displayData.bool1,
            displayData.bool2,
            displayData.bool3,
            displayData.bool4,
            displayData.bool5,
            displayData.bool6))
    };
    customData.data = data;
    customData.length = 13;
    return sendCustomData(customData);
}

void RefSerial::sendCustomData(const CustomData& customData)
{
    // Exceed max length
    if (customData.length > CUSTOM_DATA_MAX_LENGTH)
    {
        return;
    }
    // Check if sender and recipient is from our alliance
    // trying to send to red and robot is actually blue
    if (customData.senderId < BLUE_HERO && customData.recipientId > BLUE_HERO)
    {
        return;
    }
    // Check if sender and recipient is from our alliance
    // trying to send to a blue robot and robot is actually red
    if (robotData.robotId >= BLUE_HERO && customData.recipientId < BLUE_HERO)
    {
        return;
    }

    if (aruwlib::arch::clock::getTimeMilliseconds() - this->txMessage.messageTimestamp.getTime()
        < TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS
) {
        // not enough time has passed before next send
        // send at max every 100 ms (max frequency 10Hz)
        return;
    }

    this->txMessage.type = REF_MESSAGE_TYPE_CUSTOM_DATA;
    this->txMessage.length = customData.length
        + CUSTOM_DATA_TYPE_LENGTH
        + CUSTOM_DATA_SENDER_ID_LENGTH
        + CUSTOM_DATA_RECIPIENT_ID_LENGTH;

    // this message consists of the following:
    // - custom data type
    // - custom data sender id
    // - custom data receipent id
    // this is all stored in the message itself, in addition to the message data
    this->txMessage.data[0] = static_cast<uint8_t>(customData.type);
    this->txMessage.data[1] = static_cast<uint8_t>(customData.type >> 8);
    this->txMessage.data[CUSTOM_DATA_TYPE_LENGTH] = static_cast<uint8_t>(customData.senderId);
    this->txMessage.data[CUSTOM_DATA_TYPE_LENGTH + 1] =
        static_cast<uint8_t>(customData.senderId >> 8);
    this->txMessage.data[CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH] =
        static_cast<uint8_t>(customData.recipientId);
    this->txMessage.data[CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH + 1] =
        static_cast<uint8_t>(customData.recipientId >> 8);
    memcpy(this->txMessage.data + CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH
        + CUSTOM_DATA_RECIPIENT_ID_LENGTH, customData.data, customData.length);

    this->send();
}

uint8_t RefSerial::packBoolMask(
    bool bool1,
    bool bool2,
    bool bool3,
    bool bool4,
    bool bool5,
    bool bool6
) {
    return static_cast<uint8_t>(bool1) |
        static_cast<uint8_t>(bool2) << 1 |
        static_cast<uint8_t>(bool3) << 2 |
        static_cast<uint8_t>(bool4) << 3 |
        static_cast<uint8_t>(bool5) << 4 |
        static_cast<uint8_t>(bool6) << 5;  // bits 6 and 7 are reserved by the ref system
}

uint16_t RefSerial::getRobotClientID(RobotId robotId)
{
    // there are no client_id for sentinel robots because there are no ui display for them
    if (robotId == RED_SENTINEL || robotId == BLUE_SENTINEL)
    {
        return 0;
    }
    uint16_t convertedRobotId = 0x100;
    if (robotId > 10)
    {  // if robotId is a blue robot
        convertedRobotId += 6;
    }
    return convertedRobotId + (uint16_t) robotId;
}

const RefSerial::RobotData& RefSerial::getRobotData() const
{
    return robotData;
}

const RefSerial::GameData& RefSerial::getGameData() const
{
    return gameData;
}

float RefSerial::decodeTofloat(const uint8_t* startByte)
{
    uint32_t unsigned_value = (
        startByte[3] << 24)
        | (startByte[2] << 16)
        | (startByte[1] << 8)
        | startByte[0];
    return aruwlib::algorithms::reinterpretCopy<uint32_t, float>(unsigned_value);
}

bool RefSerial::decodeToGameStatus(const SerialMessage& message)
{
    if (message.length != 3)
    {
        return false;
    }
    gameData.gameStage = static_cast<GameStages>(message.data[0] >> 4);
    gameData.stageTimeRemaining = (message.data[2] << 8) | message.data[1];
    return true;
}

bool RefSerial::decodeToGameResult(const SerialMessage& message)
{
    if (message.length != 1)
    {
        return false;
    }
    gameData.gameWinner = static_cast<GameWinner>(message.data[0]);
    return true;
}

bool RefSerial::decodeToAllRobotHP(const SerialMessage& message)
{
    if (message.length != 28)  // todo
    {
        return false;
    }
    robotData.allRobotHp.redHero
        = (message.data[1] << 8) | message.data[0];
    robotData.allRobotHp.redEngineer
        = (message.data[3] << 8) | message.data[2];
    robotData.allRobotHp.redSoldier1
        = (message.data[5] << 8) | message.data[4];
    robotData.allRobotHp.redSoldier2
        = (message.data[7] << 8) | message.data[6];
    robotData.allRobotHp.redSoldier3
        = (message.data[9] << 8) | message.data[8];
    robotData.allRobotHp.redSentinel
        = (message.data[11] << 8) | message.data[10];
    robotData.allRobotHp.redBase
        = (message.data[13] << 8) | message.data[12];
    robotData.allRobotHp.blueHero
        = (message.data[15] << 8) | message.data[14];
    robotData.allRobotHp.blueEngineer
        = (message.data[17] << 8) | message.data[16];
    robotData.allRobotHp.blueSoldier1
        = (message.data[19] << 8) | message.data[18];
    robotData.allRobotHp.blueSoldier2
        = (message.data[21] << 8) | message.data[20];
    robotData.allRobotHp.blueSoldier3
        = (message.data[23] << 8) | message.data[22];
    robotData.allRobotHp.blueSentinel
        = (message.data[25] << 8) | message.data[24];
    robotData.allRobotHp.blueBase
        = (message.data[27] << 8) | message.data[26];
    return true;
}

bool RefSerial::decodeToRobotStatus(const SerialMessage& message)
{
    if (message.length != 15)
    {
        return false;
    }
    robotData.robotId = (RobotId) message.data[0];
    robotData.robotLevel = message.data[1];
    robotData.currentHp = (message.data[3] << 8) | message.data[2];
    robotData.maxHp = (message.data[5] << 8) | message.data[4];
    robotData.turret.heatCoolingRate17
        = (message.data[7] << 8) | message.data[6];
    robotData.turret.heatLimit17
        = (message.data[9] << 8) | message.data[8];
    robotData.turret.heatCoolingRate42
        = (message.data[11] << 8) | message.data[10];
    robotData.turret.heatLimit42
        = (message.data[13] << 8) | message.data[12];
    robotData.gimbalHasPower = message.data[14];
    robotData.chassisHasPower = (message.data[14] >> 1);
    robotData.shooterHasPower = (message.data[14] >> 2);

    processReceivedDamage(message.messageTimestamp.getTime(), robotData.previousHp
        - robotData.currentHp);
    robotData.previousHp = robotData.currentHp;

    return true;
}



bool RefSerial::decodeToPowerAndHeat(const SerialMessage& message)
{
    if (message.length != 14)
    {
        return false;
    }
    robotData.chassis.volt = (message.data[1] << 8) | message.data[0];
    robotData.chassis.current = (message.data[3] << 8) | message.data[2];
    robotData.chassis.power = RefSerial::decodeTofloat(&message.data[4]);
    robotData.chassis.powerBuffer = (message.data[9] << 8) | message.data[8];
    robotData.turret.heat17 = (message.data[11] << 8) | message.data[10];
    robotData.turret.heat42 = (message.data[13] << 8) | message.data[12];
    return true;
}

bool RefSerial::decodeToRobotPosition(const SerialMessage& message)
{
    if (message.length != 16)
    {
        return false;
    }
    robotData.chassis.x = RefSerial::decodeTofloat(&message.data[0]);
    robotData.chassis.y = RefSerial::decodeTofloat(&message.data[4]);
    robotData.chassis.z = RefSerial::decodeTofloat(&message.data[8]);
    robotData.turret.yaw = RefSerial::decodeTofloat(&message.data[12]);
    return true;
}

bool RefSerial::decodeToReceiveDamage(const SerialMessage& message)
{
    if (message.length != 1)
    {
        return false;
    }
    robotData.damagedArmorId = (ArmorId) message.data[0];
    robotData.damageType = (DamageType) (message.data[0] >> 4);
    robotData.previousHp = robotData.currentHp;
    return true;
}

bool RefSerial::decodeToProjectileLaunch(const SerialMessage& message)
{
    if (message.length != 6)
    {
        return false;
    }
    robotData.turret.bulletType = (BulletType) message.data[0];
    robotData.turret.firing_freq = message.data[1];
    robotData.turret.bulletSpeed = RefSerial::decodeTofloat(&message.data[2]);
    return true;
}

bool RefSerial::decodeToSentinelDroneBulletsRemain(
    const SerialMessage& message
) {
    if (message.length != 2)
    {
        return false;
    }
    robotData.turret.sentinelDroneBulletsRemain
        = (message.data[1] << 8) | message.data[0];
    return true;
}

void RefSerial::processReceivedDamage(uint32_t timestamp, int32_t damageTaken)
{
    if (damageTaken > 0)
    {
        // create a new DamageEvent with the damage_taken, and current time
        DamageEvent damageEvent = {
            static_cast<uint16_t>(damageTaken),
            timestamp
        };

        if (receivedDpsTracker.getSize() == REF_DAMAGE_EVENT_SIZE) {
            receivedDpsTracker.removeBack();
        }
        robotData.receivedDps += damageTaken;

        receivedDpsTracker.append(damageEvent);
    }
}

void RefSerial::updateReceivedDamage()
{
    // if current damage at head of circular array occurred more than a second ago,
    // decrease receivedDps by that amount of damage and increment head index
    while (
        receivedDpsTracker.getSize() > 0
        && aruwlib::arch::clock::getTimeMilliseconds() -
        receivedDpsTracker.getFront().timestampMs > 1000
    ) {
        robotData.receivedDps -= receivedDpsTracker.getFront().damageAmount;
        receivedDpsTracker.removeFront();
    }
}

}  // namespace serial

}  // namespace aruwlib
