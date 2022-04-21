/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;
using namespace tap;
using namespace testing;

class RefSerialTransmitterTest : public Test
{
protected:
    RefSerialTransmitterTest() : refSerialTransmitter(&drivers) {}

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRobotData()).WillByDefault(ReturnRef(robotData));
    }

    Drivers drivers;
    RefSerialTransmitter refSerialTransmitter;
    RefSerialData::Rx::RobotData robotData{};
};

TEST_F(RefSerialTransmitterTest, configGraphicGenerics__sets_name_operation_layer_color)
{
    RefSerial::Tx::GraphicData data;

    uint8_t name[3] = {0, 1, 0};

    RefSerialTransmitter::configGraphicGenerics(
        &data,
        name,
        RefSerial::Tx::GraphicOperation::GRAPHIC_MODIFY,
        0,
        RefSerial::Tx::GraphicColor::PINK);

    EXPECT_TRUE(0 == std::memcmp(name, data.name, sizeof(name)));
    EXPECT_EQ(RefSerial::Tx::GraphicOperation::GRAPHIC_MODIFY, data.operation);
    EXPECT_EQ(static_cast<uint8_t>(RefSerial::Tx::GraphicColor::PINK), data.color);
}

TEST_F(RefSerialTransmitterTest, deleteGraphicLayer__doesnt_send_if_robot_id_invalid)
{
    RefSerial::Tx::DeleteGraphicOperation op =
        RefSerial::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER;

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerialTransmitter.deleteGraphicLayer(op, 1);
}

TEST_F(RefSerialTransmitterTest, deleteGraphicLayer__sends_correct_msg)
{
    robotData.robotId = RefSerial::RobotId::RED_SOLDIER_3;

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_))
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            const RefSerial::Tx::DeleteGraphicLayerMessage *msg =
                reinterpret_cast<const RefSerial::Tx::DeleteGraphicLayerMessage *>(data);

            EXPECT_EQ(
                sizeof(msg->interactiveHeader) + sizeof(msg->deleteOperation) + sizeof(msg->layer),
                msg->frameHeader.dataLength);
            uint16_t cmdId = *reinterpret_cast<const uint16_t *>(data + sizeof(msg->frameHeader));
            EXPECT_EQ(0x0301, cmdId);
            EXPECT_EQ(0xa5, msg->frameHeader.headByte);
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

    refSerialTransmitter.deleteGraphicLayer(
        RefSerial::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER,
        2);
}

TEST_F(RefSerialTransmitterTest, sendGraphic__1_doesnt_send_if_robot_id_invalid)
{
    RefSerial::Tx::Graphic1Message msg{};

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerialTransmitter.sendGraphic(&msg);
}

TEST_F(RefSerialTransmitterTest, sendGraphic__1_doesnt_sen_but_configures_if_sendMsg_false)
{
    robotData.robotId = RefSerial::RobotId::BLUE_SOLDIER_1;

    RefSerial::Tx::Graphic1Message msg{};

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerialTransmitter.sendGraphic(&msg, true, false);

    // validate the msg header was still constructed
    EXPECT_EQ(0x0301, msg.cmdId);
    EXPECT_EQ(sizeof(msg.graphicData) + sizeof(msg.interactiveHeader), msg.frameHeader.dataLength);
    EXPECT_EQ(0xa5, msg.frameHeader.headByte);
    EXPECT_EQ(
        static_cast<uint16_t>(RefSerial::RobotId::BLUE_SOLDIER_1),
        msg.interactiveHeader.senderId);
    EXPECT_EQ(
        0x100 + static_cast<uint16_t>(RefSerial::RobotId::BLUE_SOLDIER_1),
        msg.interactiveHeader.receiverId);
}

TEST_F(
    RefSerialTransmitterTest,
    sendGraphic__1_doesnt_send_or_config_if_configMsgHeader_sendMsg_false)
{
    robotData.robotId = RefSerial::RobotId::BLUE_SOLDIER_1;

    RefSerial::Tx::Graphic1Message msg{};
    msg.cmdId = 0x1;

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerialTransmitter.sendGraphic(&msg, false, false);

    // validate the msg header was still constructed
    EXPECT_EQ(0x1, msg.cmdId);
}

TEST_F(RefSerialTransmitterTest, sendGraphic__characterMessage)
{
    robotData.robotId = RefSerial::RobotId::BLUE_ENGINEER;

    RefSerial::Tx::GraphicCharacterMessage msg{};

    msg.msg[0] = 'h';
    msg.msg[1] = 'e';
    msg.msg[2] = 'l';
    msg.msg[3] = 'l';
    msg.msg[4] = 'o';
    msg.msg[5] = '\0';

    EXPECT_CALL(
        drivers.uart,
        write(testing::_, testing::_, sizeof(RefSerial::Tx::GraphicCharacterMessage)))
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            const auto header = reinterpret_cast<const RefSerial::FrameHeader *>(data);
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

    refSerialTransmitter.sendGraphic(&msg);
}

TEST_F(RefSerialTransmitterTest, sendRobotToRobotMessage__invalid_id_fails_to_send)
{
    robotData.robotId = RefSerial::RobotId::INVALID;

    RefSerial::Tx::RobotToRobotMessage msg{};

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x01ff, RefSerial::RobotId::RED_HERO, 2);
    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x1000, RefSerial::RobotId::RED_HERO, 2);
}

TEST_F(
    RefSerialTransmitterTest,
    sendRobotToRobotMessage__validate_sending_msg_to_same_color_robot_works)
{
    robotData.robotId = RefSerial::RobotId::RED_DRONE;

    RefSerial::Tx::RobotToRobotMessage msg;

    msg.dataAndCRC16[0] = 'h';
    msg.dataAndCRC16[1] = 'i';

    static constexpr int msgLen = 2;

    static constexpr int entireMsgLen = sizeof(msg.frameHeader) + sizeof(msg.cmdId) +
                                        sizeof(msg.interactiveHeader) + msgLen + sizeof(uint16_t);

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, entireMsgLen))
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            // Decode and validate header
            const RefSerial::FrameHeader *header =
                reinterpret_cast<const RefSerial::FrameHeader *>(data);
            EXPECT_EQ(sizeof(msg.interactiveHeader) + msgLen, header->dataLength);
            EXPECT_EQ(0xa5, header->headByte);
            EXPECT_EQ(
                tap::algorithms::calculateCRC8(data, sizeof(RefSerial::FrameHeader) - 1),
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

    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x0200, RefSerial::RobotId::RED_HERO, 2);
}
