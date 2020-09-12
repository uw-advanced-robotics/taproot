/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "dji_serial.hpp"

#include "aruwlib/algorithms/crc.hpp"
#include "aruwlib/communication/serial/uart.hpp"
#include "aruwlib/errors/create_errors.hpp"

namespace aruwlib
{
namespace serial
{
DJISerial::DJISerial(Drivers *drivers, Uart::UartPort port, bool isRxCRCEnforcementEnabled)
    : port(port),
      djiSerialRxState(SERIAL_HEADER_SEARCH),
      frameCurrReadByte(0),
      frameHeader(),
      rxCRCEnforcementEnabled(isRxCRCEnforcementEnabled),
      txBuffer(),
      drivers(drivers)
{
    txMessage.length = 0;
    newMessage.length = 0;
    mostRecentMessage.length = 0;
}

void DJISerial::initialize()
{
    switch (this->port)
    {
        case Uart::UartPort::Uart1:
            drivers->uart.init<Uart::UartPort::Uart1, 115200>();
            break;
        case Uart::UartPort::Uart2:
            drivers->uart.init<Uart::UartPort::Uart2, 115200>();
            break;
        case Uart::UartPort::Uart6:
            drivers->uart.init<Uart::UartPort::Uart6, 115200>();
            break;
        default:
            break;
    }
}

bool DJISerial::send()
{
    txBuffer[0] = SERIAL_HEAD_BYTE;
    txBuffer[FRAME_DATA_LENGTH_OFFSET] = txMessage.length & 0xFF;
    txBuffer[FRAME_DATA_LENGTH_OFFSET + 1] = (txMessage.length >> 8) & 0xFF;
    txBuffer[FRAME_SEQUENCENUM_OFFSET] = txMessage.sequenceNumber;
    txBuffer[FRAME_CRC8_OFFSET] = algorithms::calculateCRC8(txBuffer, 4);
    txBuffer[FRAME_TYPE_OFFSET] = (txMessage.type) & 0xFF;
    txBuffer[FRAME_TYPE_OFFSET + 1] = (txMessage.type >> 8) & 0xFF;

    // we can't send, trying to send too much
    if (FRAME_HEADER_LENGTH + txMessage.length + FRAME_CRC16_LENGTH >= SERIAL_TX_BUFF_SIZE)
    {
        RAISE_ERROR(
            drivers,
            "dji serial attempting to send greater than SERIAL_TX_BUFF_SIZE bytes",
            aruwlib::errors::Location::DJI_SERIAL,
            aruwlib::errors::ErrorType::MESSAGE_LENGTH_OVERFLOW);
        return false;
    }

    memcpy(txBuffer + FRAME_HEADER_LENGTH, txMessage.data, txMessage.length);

    // add crc16
    uint16_t CRC16Val =
        algorithms::calculateCRC16(txBuffer, FRAME_HEADER_LENGTH + txMessage.length);
    txBuffer[FRAME_HEADER_LENGTH + txMessage.length] = CRC16Val;
    txBuffer[FRAME_HEADER_LENGTH + txMessage.length + 1] = CRC16Val >> 8;

    uint32_t totalSize = FRAME_HEADER_LENGTH + txMessage.length + FRAME_CRC16_LENGTH;
    uint32_t messageLengthSent = this->write(txBuffer, totalSize);
    if (messageLengthSent != totalSize)
    {
        return false;
        // the message did not completely send
        RAISE_ERROR(
            drivers,
            "the message did not completely send",
            aruwlib::errors::Location::DJI_SERIAL,
            aruwlib::errors::ErrorType::INVALID_MESSAGE_LENGTH);
    }
    txMessage.messageTimestamp = modm::Clock::now();
    return true;
}

void DJISerial::updateSerial()
{
    switch (djiSerialRxState)
    {
        case SERIAL_HEADER_SEARCH:
        {
            // keep scanning for the head byte as long as you are here and have not yet found it.
            uint8_t serialHeadCheck = 0;
            while (djiSerialRxState == SERIAL_HEADER_SEARCH && read(&serialHeadCheck, 1))
            {
                // we found it, store the head byte
                if (serialHeadCheck == SERIAL_HEAD_BYTE)
                {
                    frameHeader[0] = SERIAL_HEAD_BYTE;
                    newMessage.headByte = SERIAL_HEAD_BYTE;
                    djiSerialRxState = PROCESS_FRAME_HEADER;
                }
            }

            break;
        }
        case PROCESS_FRAME_HEADER:  // the frame header consists of the length, type, and CRC8
        {
            // Read from the buffer. Keep track of the index in the frameHeader array using the
            // frameCurrReadByte. +1 at beginning and -1 on the end since the serial head
            // byte is part of the frame but has already been processed.
            frameCurrReadByte += read(
                frameHeader + frameCurrReadByte + 1,
                FRAME_HEADER_LENGTH - frameCurrReadByte - 1);

            // We have the complete message header in the frameHeader buffer
            if (frameCurrReadByte == FRAME_HEADER_LENGTH - 1)
            {
                frameCurrReadByte = 0;

                // process length
                newMessage.length = (frameHeader[FRAME_DATA_LENGTH_OFFSET + 1] << 8) |
                                    frameHeader[FRAME_DATA_LENGTH_OFFSET];
                // process sequence number (counter)
                newMessage.sequenceNumber = frameHeader[FRAME_SEQUENCENUM_OFFSET];
                newMessage.type =
                    frameHeader[FRAME_TYPE_OFFSET + 1] << 8 | frameHeader[FRAME_TYPE_OFFSET];

                if (newMessage.length == 0 ||
                    newMessage.length >=
                        SERIAL_RX_BUFF_SIZE - (FRAME_HEADER_LENGTH + FRAME_CRC16_LENGTH))
                {
                    djiSerialRxState = SERIAL_HEADER_SEARCH;
                    RAISE_ERROR(
                        drivers,
                        "invalid message length received",
                        aruwlib::errors::Location::DJI_SERIAL,
                        aruwlib::errors::ErrorType::INVALID_MESSAGE_LENGTH);
                    return;
                }

                // check crc8 on header
                if (rxCRCEnforcementEnabled)
                {
                    uint8_t CRC8 = frameHeader[FRAME_CRC8_OFFSET];
                    // don't look at crc8 or frame type when calculating crc8
                    if (!verifyCRC8(frameHeader, FRAME_HEADER_LENGTH - 3, CRC8))
                    {
                        djiSerialRxState = SERIAL_HEADER_SEARCH;
                        RAISE_ERROR(
                            drivers,
                            "CRC8 failure",
                            aruwlib::errors::Location::DJI_SERIAL,
                            aruwlib::errors::ErrorType::CRC_FAILURE);
                        return;
                    }
                }

                // move on to processing message body
                djiSerialRxState = PROCESS_FRAME_DATA;
            }
            break;
        }
        case PROCESS_FRAME_DATA:  // read bulk of message
        {
            // add on extra 2 bytes for crc enforcement, and read bytes until
            // the length has been reached
            if (rxCRCEnforcementEnabled)
            {
                frameCurrReadByte += read(
                    newMessage.data + frameCurrReadByte,
                    newMessage.length + 2 - frameCurrReadByte);
            }
            else
            {
                frameCurrReadByte += read(
                    newMessage.data + frameCurrReadByte,
                    newMessage.length - frameCurrReadByte);
            }

            if ((frameCurrReadByte == newMessage.length && !rxCRCEnforcementEnabled) ||
                (frameCurrReadByte == newMessage.length + 2 && rxCRCEnforcementEnabled))
            {
                frameCurrReadByte = 0;
                if (rxCRCEnforcementEnabled)
                {
                    uint8_t *crc16CheckData =
                        new uint8_t[FRAME_HEADER_LENGTH + newMessage.length]();
                    memcpy(crc16CheckData, frameHeader, FRAME_HEADER_LENGTH);
                    memcpy(
                        crc16CheckData + FRAME_HEADER_LENGTH,
                        newMessage.data,
                        newMessage.length);

                    uint16_t CRC16 = (newMessage.data[newMessage.length + 1] << 8) |
                                     newMessage.data[newMessage.length];
                    if (!verifyCRC16(
                            crc16CheckData,
                            FRAME_HEADER_LENGTH + newMessage.length,
                            CRC16))
                    {
                        delete[] crc16CheckData;
                        djiSerialRxState = SERIAL_HEADER_SEARCH;
                        RAISE_ERROR(
                            drivers,
                            "CRC16 failure",
                            aruwlib::errors::Location::DJI_SERIAL,
                            aruwlib::errors::ErrorType::CRC_FAILURE);
                        return;
                    }
                    delete[] crc16CheckData;
                }

                // update the time and copy over the message to the most recent message
                newMessage.messageTimestamp = modm::Clock::now();

                mostRecentMessage = newMessage;

                messageReceiveCallback(mostRecentMessage);

                djiSerialRxState = SERIAL_HEADER_SEARCH;
            }
            else if (
                (frameCurrReadByte > newMessage.length && !rxCRCEnforcementEnabled) ||
                (frameCurrReadByte > newMessage.length + 2 && rxCRCEnforcementEnabled))
            {
                frameCurrReadByte = 0;
                RAISE_ERROR(
                    drivers,
                    "Invalid message length",
                    aruwlib::errors::Location::DJI_SERIAL,
                    aruwlib::errors::ErrorType::INVALID_MESSAGE_LENGTH);
                djiSerialRxState = SERIAL_HEADER_SEARCH;
            }
            break;
        }
    }
}

bool DJISerial::verifyCRC8(uint8_t *data, uint32_t length, uint8_t expectedCRC8)
{
    uint8_t actualCRC8 = 0;
    if (data == NULL)
    {
        return false;
    }
    actualCRC8 = algorithms::calculateCRC8(data, length);
    return actualCRC8 == expectedCRC8;
}

bool DJISerial::verifyCRC16(uint8_t *data, uint32_t length, uint16_t expectedCRC16)
{
    uint16_t actualCRC16 = 0;
    if (data == NULL)
    {
        return false;
    }
    actualCRC16 = algorithms::calculateCRC16(data, length);
    return actualCRC16 == expectedCRC16;
}

uint32_t DJISerial::read(uint8_t *data, uint16_t length)
{
    return drivers->uart.read(this->port, data, length);
}

uint32_t DJISerial::write(const uint8_t *data, uint16_t length)
{
    if (drivers->uart.isWriteFinished(this->port))
    {
        return drivers->uart.write(this->port, data, length);
    }
    else
    {
        return 0;
    }
}

}  // namespace serial

}  // namespace aruwlib
