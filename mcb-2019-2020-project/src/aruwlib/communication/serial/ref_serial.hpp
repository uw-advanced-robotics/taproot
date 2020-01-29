#ifndef __REF_SERIAL_HPP__
#define __REF_SERIAL_HPP__

#include <modm/container/deque.hpp>
#include <rm-dev-board-a/board.hpp>
#include "dji_serial.hpp"

namespace aruwlib
{

namespace serial
{

class RefSerial : public DJISerial
{
 private:
    // RX message constants
    static const uint16_t REF_DAMAGE_EVENT_SIZE = 20;

    static const uint16_t CUSTOM_DATA_MAX_LENGTH = 113;
    static const uint16_t CUSTOM_DATA_TYPE_LENGTH = 2;
    static const uint16_t CUSTOM_DATA_SENDER_ID_LENGTH = 2;
    static const uint16_t CUSTOM_DATA_RECIPIENT_ID_LENGTH = 2;

    // tx message constants
    static const uint32_t TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS = 100;

    // rx message type defines
    static const uint16_t REF_MESSAGE_TYPE_GAME_STATUS                   = 0x1;
    static const uint16_t REF_MESSAGE_TYPE_GAME_RESULT                   = 0x2;
    static const uint16_t REF_MESSAGE_TYPE_ALL_ROBOT_HP                  = 0x3;
    static const uint16_t REF_MESSAGE_TYPE_ROBOT_STATUS                  = 0x201;
    static const uint16_t REF_MESSAGE_TYPE_POWER_AND_HEAT                = 0x202;
    static const uint16_t REF_MESSAGE_TYPE_ROBOT_POSITION                = 0x203;
    static const uint16_t REF_MESSAGE_TYPE_RECEIVE_DAMAGE                = 0x206;
    static const uint16_t REF_MESSAGE_TYPE_PROJECTILE_LAUNCH             = 0x207;
    static const uint16_t REF_MESSAGE_TYPE_SENTINEL_DRONE_BULLETS_REMAIN = 0x208;
    static const uint16_t REF_MESSAGE_TYPE_CUSTOM_DATA                   = 0x301;

    // rx message type defines
    static const uint16_t REF_CUSTOM_DATA_TYPE_UI_INDICATOR = 0xD180;

 public:
    typedef enum
    {
        PREMATCH = 0,        // pre-competition stage
        SETUP = 1,           // setup stage
        INITIALIZATION = 2,  // initialization stage
        COUNTDOWN = 3,       // 5-second countdown
        IN_GAME = 4,         // in middle of the game
        END_GAME = 5,        // calculating competition results
    } GameStages;

    typedef enum
    {
        DRAW = 0,  // match was a draw
        RED = 1,   // red team won the match
        BLUE = 2,  // blue team won the match
    } GameWinner;

    typedef enum
    {
        RED_HERO = 1,
        RED_ENGINEER = 2,
        RED_SOLDIER_1 = 3,
        RED_SOLDIER_2 = 4,
        RED_SOLDIER_3 = 5,
        RED_DRONE = 6,
        RED_SENTINEL = 7,

        BLUE_HERO = 11,
        BLUE_ENGINEER = 12,
        BLUE_SOLDIER_1 = 13,
        BLUE_SOLDIER_2 = 14,
        BLUE_SOLDIER_3 = 15,
        BLUE_DRONE = 16,
        BLUE_SENTINEL = 17,
    } RobotId;

    typedef enum
    {
        FRONT = 0,  // armor #0 (front)
        LEFT = 1,   // armor #1 (left)
        REAR = 2,   // armor #2 (rear)
        RIGHT = 3,  // armor #3 (right)
        TOP = 4,    // armor #4 (top)
    } ArmorId;

    typedef enum
    {
        NO_DAMAGE_RECEIVED = 0,
        MODULE_OFFLINE = 1,         // module offline
        ARMOR_DAMAGE = 2,           // armor damage
        BARREL_OVERHEAT = 3,        // barrel overheat
        CHASSIS_POWER_OVERRUN = 4,  // chassis power overrun
        COLLISION = 5,              // chassis collision
    } DamageType;

    typedef struct
    {
        uint16_t damageAmount;  // amount of damage received
        uint32_t timestampMs;   // time when damage was received (in milliseconds)
    } DamageEvent;

    typedef enum
    {
        AMMO_17 = 1,  // 17 mm projectile ammo
        AMMO_42 = 2,  // 42 mm projectile ammo
    } BulletType;

    typedef struct
    {
        GameStages gameStage : 4;     // current stage in the game
        uint16_t stageTimeRemaining;  // remaining time in the current stage (in seconds)
        GameWinner gameWinner;        // results of the match
    } GameData;

    typedef struct
    {
        uint16_t volt;         // output voltage to the chassis (in mV)
        uint16_t current;      // output current to the chassis (in mA)
        float power;           // output power to the chassis (in W)
        uint16_t powerBuffer;  // chassis power buffer (in J)
        float x, y, z;         // x, y, z coordinate of the chassis
    } ChassisData;

    typedef struct
    {
        BulletType bulletType;                // 17mm or 42mm last projectile shot
        uint8_t firing_freq;                  // firing frequency (in Hz)
        uint16_t heat17;                      // current 17mm turret heat
        uint16_t heatCoolingRate17;           // 17mm turret cooling value per second
        uint16_t heatLimit17;                 // 17mm turret heat limit
        uint16_t heat42;                      // current 42mm turret heat
        uint16_t heatCoolingRate42;           // 42mm turret cooling value per second
        uint16_t heatLimit42;                 // 42mm turret heat limit
        uint16_t sentinelDroneBulletsRemain;  // number of bullets remaining in sentinel
                                              // and drone only (500 max)
        float bulletSpeed;                    // last bullet speed (in m/s)
        float yaw;                            // barrel yaw position (degree)
    } TurretData;

    typedef struct
    {
        // current HP of all robots
        uint16_t redHero;
        uint16_t redEngineer;
        uint16_t redSoldier1;
        uint16_t redSoldier2;
        uint16_t redSoldier3;
        uint16_t redSentinel;
        uint16_t redBase;
        uint16_t blueHero;
        uint16_t blueEngineer;
        uint16_t blueSoldier1;
        uint16_t blueSoldier2;
        uint16_t blueSoldier3;
        uint16_t blueSentinel;
        uint16_t blueBase;
    } RobotHpData;

    typedef struct
    {
        RobotId robotId;              // robot type and team
        uint8_t robotLevel;           // current level of this robot (1-3)
        uint16_t previousHp;          // health of this robot before damage was
                                      // received, used to calculate receivedDps
                                      // if no damage was received recently,
                                      // previousHp = currentHp
        uint16_t currentHp;           // current health of this robot
        uint16_t maxHp;               // max health of this robot
        uint8_t gimbalHasPower : 1;   // 1 if there is 24V output to gimbal, 0 for 0V
        uint8_t chassisHasPower : 1;  // 1 if there is 24V output to chassis, 0 for 0V
        uint8_t shooterHasPower : 1;  // 1 if there is 24V output to shooter, 0 for 0V
        ArmorId damagedArmorId : 4;   // armor ID that was damaged
        DamageType damageType : 4;    // cause of damage
        float receivedDps;            // damage per second received
        ChassisData chassis;          // chassis power draw and position data
        TurretData turret;            // turret firing and heat data
        RobotHpData allRobotHp;       // current HP of all the robots
    } RobotData;

    typedef struct
    {
        float float1;
        float float2;
        float float3;

        bool bool1;
        bool bool2;
        bool bool3;
        bool bool4;
        bool bool5;
        bool bool6;
    } DisplayData;

    typedef struct
    {
        uint16_t type;
        uint16_t senderId;
        uint16_t recipientId;
        uint8_t* data;
        uint16_t length;
    } CustomData;

    RefSerial();

    /**
     * Handles the types of messages defined above in the RX message handlers section
     */
    void messageReceiveCallback(SerialMessage completeMessage) override;

    const RobotData& getRobotData() const;

    const GameData& getGameData() const;

    void sendDisplayData(const DisplayData& displayData);

 private:
    RobotData robotData;
    GameData gameData;
    modm::BoundedDeque<DamageEvent, REF_DAMAGE_EVENT_SIZE> receivedDpsTracker;

    void sendCustomData(const CustomData& customData);

    /** 
     * @brief given 6 boolean variables to display to the referee ui, 
     *        packet them into an 8 bit integer and return that value.
     *        The ending bit is the first given boolean, 
     *        the next bit from the end is the second given boolean, and so on.
     * @params bool1, bool2, ..., bool6 the boolean indicator variables to display to the referee client ui.
     * @return the 8 bit variable packeting the 6 boolean indicators
     */
    uint8_t packBoolMask(
        bool bool1, bool bool2, bool bool3, bool bool4, bool bool5, bool bool6);

    /** 
     * @brief given RobotId, returns the client_id that the referee system uses to display
     *        the received messages to the given client_id robot
     * @param RobotId the id of the robot received from the referee system to get the client_id of
     * @return the client_id of the robot requested
     */
    uint16_t getRobotClientID(RobotId robotId);

    static float decodeTofloat(const uint8_t* startByte);
    bool decodeToGameStatus(const SerialMessage& message);
    bool decodeToGameResult(const SerialMessage& message);
    bool decodeToAllRobotHP(const SerialMessage& message);
    bool decodeToRobotStatus(const SerialMessage& message);
    bool decodeToPowerAndHeat(const SerialMessage& message);
    bool decodeToRobotPosition(const SerialMessage& message);
    bool decodeToReceiveDamage(const SerialMessage& message);
    bool decodeToProjectileLaunch(const SerialMessage& message);
    bool decodeToSentinelDroneBulletsRemain(const SerialMessage& message);

    void updateReceivedDamage();
    void processReceivedDamage(uint32_t timestamp, int32_t damageTaken);
};

}  // namespace serial

}  // namespace aruwlib

#endif
