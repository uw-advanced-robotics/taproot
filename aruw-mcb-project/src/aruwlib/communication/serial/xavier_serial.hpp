/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __SERIAL_XAVIER_HPP__
#define __SERIAL_XAVIER_HPP__

#include <aruwlib/architecture/timeout.hpp>

#include "dji_serial.hpp"
#include "mock_macros.hpp"

namespace aruwlib
{
class Drivers;
namespace serial
{
/**
 * A class used to communicate with our Xaviers.
 *
 * @note use the static function in Drivers to interact with this class.
 */
class XavierSerial : public DJISerial<>
{
private:
    // TX message headers.

    /// @note CV_MESSAGE_TYPEs for transmission should be defined incrementally from 0x01.

    /// Indicates you will send pitch and yaw turret data.
    static const uint8_t CV_MESSAGE_TYPE_TURRET_TELEMETRY = 0x01;
    /// Indicates you will send imu angle, gyro, and acceleration data.
    static const uint8_t CV_MESSAGE_TYPE_IMU = 0x02;
    /// Indicates you will send robot id.
    static const uint8_t CV_MESSAGE_TYPE_ROBOT_ID = 0x04;
    /// Indicates you will send a request for turret aim data.
    static const uint8_t CV_MESSAGE_TYPE_AUTO_AIM_REQUEST = 0x05;

    /// The number of message types listed above.
    static const uint8_t CV_MESSAGE_TYPE_SIZE = 4;

    // Rx related constants.

    static const int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 5000;  /// Time in ms since last CV aim data
                                                              /// was received before deciding CV
                                                              /// is offline.

    /// Indicates you will receive a turret aim request from the xavier.
    static const uint8_t CV_MESSAGE_TYPE_TURRET_AIM = 0x01;

    /// Time between each robot id send to CV in milliseconds.
    static const int16_t TIME_BETWEEN_ROBOT_ID_SEND_MS = 5000;

    // RX message constants for decoding an aim data message. These are zero indexed byte offsets.

    static const uint8_t AIM_DATA_MESSAGE_PITCH_OFFSET = 0;
    static const uint8_t AIM_DATA_MESSAGE_YAW_OFFSET = 2;
    static const uint8_t AIM_DATA_MESSAGE_HAS_TARGET = 4;
    static const uint8_t AIM_DATA_MESSAGE_SIZE = 5;

public:
    // AutoAim Data
    typedef struct
    {
        bool hasTarget;      /// Whether or not the xavier has a target.
        float pitch;         /// The pitch angle in degrees, rounded to two decimals.
        float yaw;           /// The yaw angle in degrees, rounded to two decimals.
        uint32_t timestamp;  /// A timestamp in milliseconds.
    } TurretAimData;

    /**
     * A struct that stores the imu information sent to the xavier.
     *
     * @note we round all the data here to two decimal points while sending
     *      to the xavier.
     */
    typedef struct
    {
        float ax;  /// acceleration in \f$\frac{m}{s^2}\f$.
        float ay;
        float az;

        float wx;  /// gyro values in \f$\frac{degrees}{second}\f$.
        float wy;
        float wz;

        float rol;  /// Measured in degrees.
        float pit;
        float yaw;
    } IMUData;

    /**
     * Structure for sending telemetry data to the xavier. Sends the raw
     * rpm values from the dji motors.
     */
    typedef struct
    {
        int16_t rightFrontWheelRPM;
        int16_t leftFrontWheelRPM;
        int16_t leftBackWheeRPM;
        int16_t rightBackWheelRPM;
    } ChassisData;

    XavierSerial(Drivers* drivers);
    XavierSerial(const XavierSerial&) = delete;
    XavierSerial& operator=(const XavierSerial&) = delete;
    mockable ~XavierSerial() = default;

    /**
     * Call this before using the serial line, initializes the uart line
     * and the callback
     */
    mockable void initializeCV();

    /**
     * Handles the types of messages defined above in the RX message handlers section.
     */
    void messageReceiveCallback(const SerialMessage& completeMessage) override;

    /**
     * Cycles through the messages that must be sent to the xavier.
     */
    mockable void sendMessage(
        const IMUData& imuData,
        const ChassisData& chassisData,
        const TurretAimData& turretData,
        uint8_t robotId);

    /**
     * Start Requesting Xavier to Track Target.
     */
    mockable void beginTargetTracking();

    /**
     * Stop Requesting Xavier to Track Target.
     */
    mockable void stopTargetTracking();

    /**
     * Allows the caller to extract the most up to date xavier aim data.
     *
     * @param[out] aimData a location to a `TurretAimData` struct to store the aim data in.
     * @return `true` if the xavier has sent aim data, `false` otherwise. If `false` is
     *      returned, `aimData` will not be updated.
     */
    mockable bool getLastAimData(TurretAimData* aimData) const;

private:
    // TX variables.

    /**
     * Used to increment through message send types.
     */
    uint8_t txMsgSwitchIndex;

    /**
     * Used for determining when to send robot id.
     */
    aruwlib::arch::MilliTimeout txRobotIdTimeout;

    /**
     * We queue an aim request, send it, then toggle this variable `false`
     * after the request is sent.
     */
    bool autoAimRequestQueued;

    /**
     * `true` if we are currently requesting aim data, `false` otherwise.
     */
    bool autoAimRequestState;

    // rx variables

    /**
     * The last aim data received from the xavier, used in `getLastAimData`.
     */
    TurretAimData lastAimData;

    /**
     * Whether or not aim data is up to date. Unless there has never been an
     * aim message, this is always `true`.
     *
     * \todo(Matthew) fix this state, should probably be `false` if we go offline
     *      but I should check about this.
     */
    bool hasAimData;

    // CV online variables.

    /**
     * Timer for determining if serial is offline.
     */
    aruwlib::arch::MilliTimeout cvOfflineTimeout;

    /**
     * A flag set to `true` if the timeout is not expired, and `false` otherwise.
     */
    bool isCvOnline;

    /**
     * An array that allows us to cycle through message types. Add any additional message
     * types defined above here as well.
     */
    static const uint8_t txMsgSwitchArray[CV_MESSAGE_TYPE_SIZE];

    // TX functions.

    /**
     * Interprets a raw `SerialMessage`'s `data` field to extract yaw, pitch, and other aim
     * data information.
     *
     * @param[in] message the message to be decoded.
     * @param[out] aimData a return parameter through which the decoded message is returned.
     * @return `false` if the message length doesn't match `AIM_DATA_MESSAGE_SIZE`, `true`
     *      otherwise.
     */
    bool decodeToTurrentAimData(const SerialMessage& message, TurretAimData* aimData);

    // RX functions.

    /**
     * Packages `pitch` and `yaw` data in an acceptable format for the base `DjiSerial` class
     * to interpret and sends the message via `DjiSerial`'s `send` function.
     *
     * @param[in] pitch, yaw the data to send.
     * @return `true` if sending was a success, `false` otherwise.
     */
    bool sendTurretData(float pitch, float yaw);

    /**
     * Packages `imuData` and `chassisData` data in an acceptable format for the base `DjiSerial`
     * class to interpret and sends the message via `DjiSerial`'s `send` function.
     *
     * @param[in] imuData, chassisData the data to send.
     * @return `true` if sending was a success, `false` otherwise.
     */
    bool sendIMUChassisData(const IMUData& imuData, const ChassisData& chassisData);

    /**
     * Packages `robotId` in an acceptable format for the base `DjiSerial` class to interpret
     * and sends the message via `DjiSerial`'s `send` function.
     *
     * @param robotdId the robot ID to send.
     * @return `true` if sending was a success, `false` otherwise.
     */
    bool sendRobotID(uint8_t robotId);

    /**
     * Increments `txMsgSwitchIndex`.
     */
    void incRxMsgSwitchIndex();
};

}  // namespace serial

}  // namespace aruwlib

#endif
