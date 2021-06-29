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

#include "xavier_serial.hpp"

#include <cstring>

#include "aruwlib/architecture/endianness_wrappers.hpp"
#include "aruwlib/control/chassis/i_chassis_subsystem.hpp"
#include "aruwlib/control/turret/i_turret_subsystem.hpp"
#include "aruwlib/drivers.hpp"

using namespace aruwlib::arch;
using namespace aruwlib::serial;

namespace aruwsrc
{
namespace serial
{
XavierSerial::XavierSerial(aruwlib::Drivers* drivers)
    : DJISerial(drivers, Uart::UartPort::Uart2),
      lastAimData(),
      aimDataValid(false),
      isCvOnline(false),
      turretSub(nullptr),
      chassisSub(nullptr)
{
}

void XavierSerial::initializeCV()
{
    txRobotIdTimeout.restart(TIME_BETWEEN_ROBOT_ID_SEND_MS);
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    initialize();
}

void XavierSerial::messageReceiveCallback(const SerialMessage& completeMessage)
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    switch (completeMessage.type)
    {
        case CV_MESSAGE_TYPE_TURRET_AIM:
        {
            if (decodeToTurretAimData(completeMessage, &lastAimData))
            {
                aimDataValid = true;

                if (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_SENT)
                {
                    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_COMPLETE;
                    AutoAimRequest.sendAimRequestTimeout.stop();
                }
            }
            return;
        }
        default:
            return;
    }
}

bool XavierSerial::decodeToTurretAimData(const SerialMessage& message, TurretAimData* aimData)
{
    if (message.length != AIM_DATA_MESSAGE_SIZE)
    {
        return false;
    }

    uint16_t rawPitch, rawYaw;
    convertFromLittleEndian(&rawPitch, message.data + AIM_DATA_MESSAGE_PITCH_OFFSET);
    convertFromLittleEndian(&rawYaw, message.data + AIM_DATA_MESSAGE_YAW_OFFSET);

    aimData->pitch = static_cast<float>(rawPitch) * FIXED_POINT_PRECISION;
    aimData->yaw = static_cast<float>(rawYaw) * FIXED_POINT_PRECISION;
    aimData->hasTarget = message.data[AIM_DATA_MESSAGE_HAS_TARGET_OFFSET];
    aimData->timestamp = message.messageTimestamp;

    return true;
}

bool XavierSerial::sendMessage()
{
    PT_BEGIN();
    while (true)
    {
        isCvOnline = !cvOfflineTimeout.isExpired();
        aimDataValid &= isCvOnline;
        PT_CALL(sendRobotMeasurements());
        PT_CALL(sendRobotID());
        PT_CALL(sendAutoAimRequest());
    }
    PT_END();
}

modm::ResumableResult<bool> XavierSerial::sendRobotMeasurements()
{
    RF_BEGIN(0);

    if (chassisSub != nullptr)
    {
        convertToLittleEndian(chassisSub->getRightFrontRpmActual(), txMessage.data);
        convertToLittleEndian(
            chassisSub->getLeftFrontRpmActual(),
            txMessage.data + sizeof(int16_t));
        convertToLittleEndian(
            chassisSub->getLeftBackRpmActual(),
            txMessage.data + 2 * sizeof(int16_t));
        convertToLittleEndian(
            chassisSub->getRightBackRpmActual(),
            txMessage.data + 3 * sizeof(int16_t));
    }

    if (turretSub != nullptr)
    {
        // Turret data
        convertToLittleEndian(
            static_cast<uint16_t>(
                turretSub->getCurrentPitchValue().getValue() / FIXED_POINT_PRECISION),
            txMessage.data + TURRET_DATA_OFFSET);
        convertToLittleEndian(
            static_cast<uint16_t>(
                turretSub->getCurrentYawValue().getValue() / FIXED_POINT_PRECISION),
            txMessage.data + TURRET_DATA_OFFSET + sizeof(uint16_t));
    }

    // IMU data
    convertToLittleEndian(
        static_cast<int32_t>(drivers->mpu6500.getGx() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET);
    convertToLittleEndian(
        static_cast<int32_t>(drivers->mpu6500.getGy() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + sizeof(int32_t));
    convertToLittleEndian(
        static_cast<int32_t>(drivers->mpu6500.getGz() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 2 * sizeof(int32_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getAx() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getAy() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + sizeof(int16_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getAz() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + 2 * sizeof(int16_t));
    convertToLittleEndian(
        static_cast<uint16_t>(drivers->mpu6500.getYaw() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + 3 * sizeof(int16_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getPitch() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + 4 * sizeof(int16_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getRoll() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + 5 * sizeof(int16_t));

    txMessage.type = static_cast<uint8_t>(CV_MESSAGE_TYPE_ROBOT_DATA);
    txMessage.length = ROBOT_DATA_MSG_SIZE;

    send();

    RF_YIELD();

    RF_END();
}

void XavierSerial::beginAutoAim()
{
    AutoAimRequest.autoAimRequest = true;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

void XavierSerial::stopAutoAim()
{
    AutoAimRequest.autoAimRequest = false;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

modm::ResumableResult<bool> XavierSerial::sendAutoAimRequest()
{
    RF_BEGIN(1);
    // If there is an auto aim request queued or if the request aim
    // timeout is expired (we haven't recceived a auto aim message),
    // send an auto aim request message.
    if (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_QUEUED ||
        (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_SENT &&
         AutoAimRequest.sendAimRequestTimeout.isExpired()))
    {
        txMessage.data[0] = AutoAimRequest.autoAimRequest;
        txMessage.length = 1;
        txMessage.type = CV_MESSAGE_TYPE_AUTO_AIM_REQUEST;

        if (send())
        {
            if (AutoAimRequest.autoAimRequest)
            {
                AutoAimRequest.currAimState = AUTO_AIM_REQUEST_SENT;
                AutoAimRequest.sendAimRequestTimeout.restart(AUTO_AIM_REQUEST_SEND_PERIOD_MS);
            }
            else
            {
                AutoAimRequest.currAimState = AUTO_AIM_REQUEST_COMPLETE;
            }
        }
        RF_YIELD();
    }
    RF_END();
}

modm::ResumableResult<bool> XavierSerial::sendRobotID()
{
    RF_BEGIN(2);
    if (txRobotIdTimeout.execute())
    {
        txMessage.data[0] = static_cast<uint8_t>(drivers->refSerial.getRobotData().robotId);
        txMessage.length = 1;
        txMessage.type = CV_MESSAGE_TYPE_ROBOT_ID;
        if (!send())
        {
            RF_YIELD();
        }
    }
    RF_END();
}
}  // namespace serial
}  // namespace aruwsrc
