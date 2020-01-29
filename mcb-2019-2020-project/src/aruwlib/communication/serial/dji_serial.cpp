#include <rm-dev-board-a/board.hpp>
#include "dji_serial.hpp"
#include "src/aruwlib/algorithms/crc.hpp"

namespace aruwlib
{
namespace serial
{

DJISerial::DJISerial(
    SerialPort port,
    bool isRxCRCEnforcementEnabled
):
port(port),
djiSerialRxState(SERIAL_HEADER_SEARCH),
frameCurrReadByte(0),
frameHeader(),
rxCRCEnforcementEnabled(isRxCRCEnforcementEnabled),
txBuffer()
{
    txMessage.length = 0;
    newMessage.length = 0;
    mostRecentMessage.length = 0;
}

void DJISerial::initialize() {
    switch (this->port) {
    case PORT_UART2:
        Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
        Usart2::initialize<Board::SystemClock, 115200>();
        break;
    case PORT_UART6:
        Usart6::connect<GpioG14::Tx, GpioG9::Rx>();
        Usart6::initialize<Board::SystemClock, 115200>();
        break;
    default:
        break;
    }
}

bool DJISerial::send() {
    txBuffer[0] = SERIAL_HEAD_BYTE;
    txBuffer[FRAME_DATA_LENGTH_OFFSET] = txMessage.length;
    txBuffer[FRAME_DATA_LENGTH_OFFSET + 1] = txMessage.length >> 8;
    txBuffer[FRAME_SEQUENCENUM_OFFSET] = txMessage.sequenceNumber;
    txBuffer[FRAME_CRC8_OFFSET] = algorithms::calculateCRC8(txBuffer, 4, CRC8_INIT);
    txBuffer[FRAME_TYPE_OFFSET] = txMessage.type;
    txBuffer[FRAME_TYPE_OFFSET + 1] = txMessage.type >> 8;

    // we can't send, trying to send too much
    if (FRAME_HEADER_LENGTH + txMessage.length + FRAME_CRC16_LENGTH >= SERIAL_TX_BUFF_SIZE) {
        // NON-FATAL-ERROR-CHECK
        return false;
    }

    memcpy(txBuffer + FRAME_HEADER_LENGTH, txMessage.data, txMessage.length);

    // add crc16
    uint16_t CRC16Val = algorithms::calculateCRC16(
        txBuffer,
        FRAME_HEADER_LENGTH + txMessage.length, CRC16_INIT
    );
    txBuffer[FRAME_HEADER_LENGTH + txMessage.length] = CRC16Val;
    txBuffer[FRAME_HEADER_LENGTH + txMessage.length + 1] = CRC16Val >> 8;

    uint32_t totalSize = FRAME_HEADER_LENGTH + txMessage.length + FRAME_CRC16_LENGTH;
    uint32_t messageLengthSent = this->write(txBuffer, totalSize);
    if (messageLengthSent != totalSize) {
        return false;
        // the message did not completely send, THROW-NON-FATAL-ERROR-CHECK
    }
    txMessage.messageTimestamp = modm::Clock::now();
    return true;
}

void DJISerial::updateSerial() {
    switch(djiSerialRxState)
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

            // recursively call youself if you have received the frame header. Important that we
            // don't miss any bytes coming in when polling.
            if (djiSerialRxState == PROCESS_FRAME_HEADER)
            {
                updateSerial();
            }
            break;
        }
        case PROCESS_FRAME_HEADER:  // the frame header consists of the length, type, and CRC8
        {
            // Read from the buffer. Keep track of the index in the frameHeader array using the
            // frameCurrReadByte. +1 at beginning and -1 on the end since the serial head
            // byte is part of the frame but has already been processed.
            frameCurrReadByte += read(frameHeader + frameCurrReadByte + 1,
                FRAME_HEADER_LENGTH - frameCurrReadByte - 1);

            // We have the complete message header in the frameHeader buffer
            if (frameCurrReadByte == FRAME_HEADER_LENGTH - 1)
            {
                frameCurrReadByte = 0;

                // process length
                newMessage.length = (frameHeader[FRAME_DATA_LENGTH_OFFSET + 1] << 8)
                    | frameHeader[FRAME_DATA_LENGTH_OFFSET];
                // process sequence number (counter)
                newMessage.sequenceNumber = frameHeader[FRAME_SEQUENCENUM_OFFSET];
                newMessage.type = frameHeader[FRAME_TYPE_OFFSET + 1] << 8
                    | frameHeader[FRAME_TYPE_OFFSET];

                if (
                    newMessage.length == 0 || newMessage.length >= SERIAL_RX_BUFF_SIZE
                    - (FRAME_HEADER_LENGTH + FRAME_CRC16_LENGTH)
                ) {
                    djiSerialRxState = SERIAL_HEADER_SEARCH;
                    // THROW-NON-FATAL-ERROR-CHECK
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
                        return;
                        // THROW-NON-FATAL-ERROR-CHECK
                    }
                }

                // move on to processing message body
                djiSerialRxState = PROCESS_FRAME_DATA;

                // recursively call yourself so you don't miss any data that is in the process of
                // being received.
                updateSerial();
            }
            break;
        }
        case PROCESS_FRAME_DATA:  // read bulk of message
        {
            // add on extra 2 bytes for crc enforcement, and read bytes until
            // the length has been reached
            if (rxCRCEnforcementEnabled)
            {
                frameCurrReadByte += read(newMessage.data + frameCurrReadByte,
                    newMessage.length + 2 - frameCurrReadByte);
            }
            else
            {
                frameCurrReadByte += read(newMessage.data + frameCurrReadByte,
                    newMessage.length - frameCurrReadByte);
            }

            if (
                (frameCurrReadByte == newMessage.length && !rxCRCEnforcementEnabled)
                || (frameCurrReadByte == newMessage.length + 2 && rxCRCEnforcementEnabled))
            {
                frameCurrReadByte = 0;
                if (rxCRCEnforcementEnabled)
                {
                    uint8_t* crc16CheckData =
                        new uint8_t[FRAME_HEADER_LENGTH + newMessage.length]();
                    memcpy(crc16CheckData, frameHeader, FRAME_HEADER_LENGTH);
                    memcpy(crc16CheckData + FRAME_HEADER_LENGTH,
                        newMessage.data, newMessage.length);

                    uint16_t CRC16 = (newMessage.data[newMessage.length + 1] << 8)
                        | newMessage.data[newMessage.length];
                    if (!verifyCRC16(crc16CheckData,
                        FRAME_HEADER_LENGTH + newMessage.length, CRC16))
                    {
                        delete[] crc16CheckData;
                        djiSerialRxState = SERIAL_HEADER_SEARCH;
                        return;
                        // NON-FATAL-ERROR-CHECK
                    }
                    delete[] crc16CheckData;
                }

                // update the time and copy over the message to the most recent message
                newMessage.messageTimestamp = modm::Clock::now();

                memcpy(&mostRecentMessage, &newMessage, sizeof(SerialMessage));

                messageReceiveCallback(mostRecentMessage);

                djiSerialRxState = SERIAL_HEADER_SEARCH;
            }
            else if (
                (frameCurrReadByte > newMessage.length && !rxCRCEnforcementEnabled)
                || (frameCurrReadByte > newMessage.length + 2 && rxCRCEnforcementEnabled))
            {
                frameCurrReadByte = 0;
                // THROW-NON-FATAL-ERROR-CHECK
                djiSerialRxState = SERIAL_HEADER_SEARCH;
            }
            break;
        }
    }
}

/**
 * Calculate CRC8 of given array and compare against expectedCRC8
 * @param data array to calculate CRC8
 * @param length length of array to check
 * @param expectedCRC8 expected CRC8
 * @return if the calculated CRC8 matches CRC8 given
 */
bool DJISerial::verifyCRC8(uint8_t *data, uint32_t length, uint8_t expectedCRC8) {
    uint8_t actualCRC8 = 0;
    if (data == NULL)
    {
        return false;
    }
    actualCRC8 = algorithms::calculateCRC8(data, length, CRC8_INIT);
    return actualCRC8 == expectedCRC8;
}

/**
 * Calculate CRC16 of given array and compare against expectedCRC16
 * @param data array to calculate CRC16
 * @param length length of array to check
 * @param expectedCRC16 expected CRC16
 * @return if the calculated CRC16 matches CRC16 given
 */
bool DJISerial::verifyCRC16(uint8_t *data, uint32_t length, uint16_t expectedCRC16) {
    uint16_t actualCRC16 = 0;
    if (data == NULL)
    {
        return false;
    }
    actualCRC16 = algorithms::calculateCRC16(data, length, CRC16_INIT);
    return actualCRC16 == expectedCRC16;
}

uint32_t DJISerial::read(uint8_t *data, uint16_t length) {
    switch (this->port) {
    case PORT_UART2:
    {
        uint32_t successRead = 0;
        for (int i = 0; i < length && Usart2::read(data[i]); i++) {
            successRead++;
        }
        return successRead;
    }
    case PORT_UART6:
    {
        uint32_t successRead = 0;
        for (int i = 0; i < length && Usart6::read(data[i]); i++) {
            successRead++;
        }
        return successRead;
    }
        return Usart6::read(data, length);
    default:
        return 0;
    }
}

uint32_t DJISerial::write(const uint8_t *data, uint16_t length) {
    switch (this->port) {
    case PORT_UART2:
        if (Usart2::isWriteFinished()) {
            return Usart2::write(data, length);
        } else {
            return 0;
        }
    case PORT_UART6:
        if (Usart6::isWriteFinished()) {
            return Usart6::write(data, length);
        } else {
            return 0;
        }
    default:
        return 0;
    }
}

}  // namespace serial

}  // namespace aruwlib
