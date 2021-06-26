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

#ifndef XAVIER_SERIAL_HPP_
#define XAVIER_SERIAL_HPP_

#include "aruwlib/architecture/periodic_timer.hpp"
#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/communication/serial/dji_serial.hpp"

#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"

#include "util_macros.hpp"

class XavierSerialTester;

namespace aruwlib
{
class Drivers;

namespace control::turret
{
class iTurretSubsystem;
}

namespace control::chassis
{
class iChassisSubsystem;
}

namespace serial
{
/**
 * A class used to communicate with our Xaviers.
 *
 * @note use the static function in Drivers to interact with this class.
 */
class XavierSerial : public aruwlib::serial::DJISerial, ::modm::pt::Protothread, modm::Resumable<3>
{
public:
    // AutoAim Data
    struct TurretAimData
    {
        bool hasTarget;      /// Whether or not the xavier has a target.
        float pitch;         /// The pitch angle in degrees, rounded to two decimals.
        float yaw;           /// The yaw angle in degrees, rounded to two decimals.
        uint32_t timestamp;  /// A timestamp in milliseconds.
    };

    enum AutoAimRequestState
    {
        AUTO_AIM_REQUEST_COMPLETE = 0,  /// Resting state for sending auto aim request
        AUTO_AIM_REQUEST_QUEUED,  /// Message has been queued but not yet sent requesting auto-aim
        AUTO_AIM_REQUEST_SENT,    /// Message has been sent and a reply is being waited for
    };

    enum TxMessageTypes
    {
        CV_MESSAGE_TYPE_ROBOT_DATA = 0,
        CV_MESSAGE_TYPE_ROBOT_ID,
        CV_MESSAGE_TYPE_AUTO_AIM_REQUEST,
        CV_NUM_MESSAGE_TYPES,
    };

    XavierSerial(aruwlib::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(XavierSerial);
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
     * Cycles through and sends the messages that must be sent to the xavier.
     * @note Uses protothread logic
     * @return `false` if the protothread is done (should never happen), or `true` otherwise.
     */
    mockable bool sendMessage();

    /**
     * Start Requesting Xavier to Track Target.
     */
    mockable void beginAutoAim();

    /**
     * Stop Requesting Xavier to Track Target.
     */
    mockable void stopAutoAim();

    mockable inline const TurretAimData& getLastAimData() const { return lastAimData; }

    mockable inline bool lastAimDataValid() const { return aimDataValid; }

    mockable inline void attachTurret(control::turret::iTurretSubsystem* turret)
    {
        turretSub = turret;
    }
    mockable inline void attachChassis(control::chassis::iChassisSubsystem* chassis)
    {
        chassisSub = chassis;
    }

private:
    friend class ::XavierSerialTester;

    enum RxMessageTypes
    {
        CV_MESSAGE_TYPE_TURRET_AIM = 0,
    };

    /// Time in ms since last CV aim data was received before deciding CV is offline.
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 5000;
    /// Time between each robot id send to CV in milliseconds.
    static constexpr int16_t TIME_BETWEEN_ROBOT_ID_SEND_MS = 5000;
    /// Time between auto aim requests (which are resent until aim data is sent from the xavier)
    static constexpr int16_t AUTO_AIM_REQUEST_SEND_PERIOD_MS = 1000;
    /// Precision of the floating point data sent to the Xavier.
    static constexpr float FIXED_POINT_PRECISION = 0.01f;

    // RX message constants for decoding an aim data message. These are zero indexed byte offsets.
    /// Offset for pitch angle
    static constexpr uint8_t AIM_DATA_MESSAGE_PITCH_OFFSET = 0;
    /// Offset for yaw angle
    static constexpr uint8_t AIM_DATA_MESSAGE_YAW_OFFSET = sizeof(uint16_t);
    /// Offset for whether or not cv has data
    static constexpr uint8_t AIM_DATA_MESSAGE_HAS_TARGET_OFFSET = 2 * sizeof(uint16_t);
    /// Size of entire message
    static constexpr uint8_t AIM_DATA_MESSAGE_SIZE = 2 * sizeof(uint16_t) + sizeof(uint8_t);

    // TX message constants for encoding robot data. These are zero indexed byte offsets.
    static constexpr uint8_t CHASSIS_DATA_OFFSET = 0;
    static constexpr uint8_t TURRET_DATA_OFFSET = 4 * sizeof(int16_t);
    static constexpr uint8_t IMU_DATA_OFFSET = TURRET_DATA_OFFSET + 2 * sizeof(uint16_t);
    static constexpr int ROBOT_DATA_MSG_SIZE =
        IMU_DATA_OFFSET + 3 * sizeof(int32_t) + 6 * sizeof(int16_t);

    /// Used for determining when to send robot id.
    aruwlib::arch::PeriodicMilliTimer txRobotIdTimeout;

    /// The most recent auto aim request state.
    struct
    {
        /// `false` if request to stop auto aiming, `true` if request to start auto aiming.
        bool autoAimRequest = false;
        AutoAimRequestState currAimState = AUTO_AIM_REQUEST_COMPLETE;
        /// Timer used to reset the aim request if acknowledgement has not been sent by xavier.
        aruwlib::arch::MilliTimeout sendAimRequestTimeout;
    } AutoAimRequest;

    /// The last aim data received from the xavier.
    TurretAimData lastAimData;

    /// Whether or not aim data is up to date.
    bool aimDataValid;

    // CV online variables.
    /// Timer for determining if serial is offline.
    aruwlib::arch::MilliTimeout cvOfflineTimeout;

    /// A flag set to `true` if the timeout is not expired, and `false` otherwise.
    bool isCvOnline;

    const control::turret::iTurretSubsystem* turretSub;
    const control::chassis::iChassisSubsystem* chassisSub;

    /**
     * Interprets a raw `SerialMessage`'s `data` field to extract yaw, pitch, and other aim
     * data information, and updates the `lastAimData`.
     *
     * @param[in] message the message to be decoded.
     * @param[out] aimData a return parameter through which the decoded message is returned.
     * @return `false` if the message length doesn't match `AIM_DATA_MESSAGE_SIZE`, `true`
     *      otherwise.
     */
    static bool decodeToTurretAimData(const SerialMessage& message, TurretAimData* aimData);

#ifdef ENV_UNIT_TESTS
public:
#endif
    modm::ResumableResult<bool> sendRobotMeasurements();

    /**
     * Packages `robotId` in an acceptable format for the base `DjiSerial` class to interpret
     * and sends the message via `DjiSerial`'s `send` function.
     *
     * @return `SUCCESS` if sending was a success, `FAIL` if send failed, `DID_NOT_SEND` otherwise.
     */
    modm::ResumableResult<bool> sendRobotID();

    modm::ResumableResult<bool> sendAutoAimRequest();
};
}  // namespace serial
}  // namespace aruwlib

#endif  // XAVIER_SERIAL_HPP_
