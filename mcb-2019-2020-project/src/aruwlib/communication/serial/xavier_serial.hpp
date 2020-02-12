#ifndef __SERIAL_XAVIER_HPP__
#define __SERIAL_XAVIER_HPP__

#include <rm-dev-board-a/board.hpp>
#include "dji_serial.hpp"

namespace aruwlib
{

namespace serial
{

class XavierSerial : public DJISerial
{
 private:
    // TX message headers
    // CV_MESSAGE_TYPEs for transmission should be defined incrementally from 0x01

    static const uint8_t CV_MESSAGE_TYPE_TURRET_TELEMETRY = 0x01;
    static const uint8_t CV_MESSAGE_TYPE_IMU              = 0x02;
    static const uint8_t CV_MESSAGE_TYPE_ROBOT_ID         = 0x04;
    static const uint8_t CV_MESSAGE_TYPE_AUTO_AIM_REQUEST = 0x05;

    static const uint8_t CV_MESSAGE_TYPE_SIZE = 4;

    // RX message headers
    static const int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 5000;  // time in ms since last CV aim data
                                                              // was received before deciding CV
                                                              // is offline

    static const uint8_t CV_MESSAGE_TYPE_TURRET_AIM = 0x01;

    // time between each robot id send to CV in milliseconds
    static const int16_t TIME_BETWEEN_ROBOT_ID_SEND_MS = 5000;

    // RX message constants for decoding an aim data message
    static const uint8_t AIM_DATA_MESSAGE_PITCH_OFFSET = 0;
    static const uint8_t AIM_DATA_MESSAGE_YAW_OFFSET   = 2;
    static const uint8_t AIM_DATA_MESSAGE_HAS_TARGET   = 4;
    static const uint8_t AIM_DATA_MESSAGE_SIZE         = 5;

 public:
    // AutoAim Data
    typedef struct
    {
        bool hasTarget;
        float pitch;
        float yaw;
        modm::Timestamp timestamp;
    } TurretAimData;

    // DO reference this struct if you want send imu data
    typedef struct
    {
        float ax;  // acceleration
        float ay;
        float az;

        float wx;  // gyro values
        float wy;
        float wz;

        float rol;  // measured in degrees
        float pit;
        float yaw;
    } IMUData;

    typedef struct
    {
        int16_t rightFrontWheelRPM;
        int16_t leftFrontWheelRPM;
        int16_t leftBackWheeRPM;
        int16_t rightBackWheelRPM;
    } ChassisData;

    XavierSerial();

    static XavierSerial& getXavierSerial();

    /**
     * Call this before using the serial line, initializes the uart line
     * and the callback
     */
    void initializeCV();

    /**
     * Handles the types of messages defined above in the RX message handlers section
     */
    void messageReceiveCallback(SerialMessage completeMessage) override;

    /**
     * Call every 1 ms
     */
    void sendMessage(
        const IMUData *imuData,
        const ChassisData *chassisData,
        const TurretAimData *turretData,
        uint8_t robotId
    );

    // Start Requesting Xavier to Track Target
    void beginTargetTracking();

    // Stop Requesting Xavier to Track Target
    void stopTargetTracking();

    bool getLastAimData(TurretAimData *aimData) const;

 private:
    // main xavier serial instance that everyone will use
    static XavierSerial xavierSerial;

    // tx variables
    uint8_t txMsgSwitchIndex;

    modm::ShortTimeout txRobotIdTimeout;

    bool autoAimRequestQueued;

    bool autoAimRequestState;

    // rx variables
    TurretAimData lastAimData;

    bool hasAimData;

    // cv online variables
    modm::ShortTimeout cvOfflineTimeout;

    bool isCvOnline;

    uint8_t txMsgSwitchArray[CV_MESSAGE_TYPE_SIZE] = {
        CV_MESSAGE_TYPE_TURRET_TELEMETRY,
        CV_MESSAGE_TYPE_IMU,
        CV_MESSAGE_TYPE_ROBOT_ID,
        CV_MESSAGE_TYPE_AUTO_AIM_REQUEST
    };

    // tx functions
    bool decodeToTurrentAimData(SerialMessage message, TurretAimData *aimData);

    // rx messages
    bool sendTurrentData(float pitch, float yaw);

    bool sendIMUChassisData(const IMUData *imuData, const ChassisData *chassisData);

    bool sendRobotID(uint8_t robotId);

    void incRxMsgSwitchIndex(void);
};

}  // namespace serial

}  // namespace aruwlib

#endif
