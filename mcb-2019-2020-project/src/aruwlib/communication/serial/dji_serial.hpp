#ifndef __serial_h_
#define __serial_h_

#include <modm/processing.hpp>
#include <rm-dev-board-a/board.hpp>

/**
 * Extend this class and implement messageReceiveCallback if you
 * want to use this serial protocol on a serial line.
 *
 * Structure of a Serial Message:
 * Frame Head{
 * [Frame Head Byte(0xA5) 1 Byte]
 * [Frame Data Length (MSB Side Byte second, LSB first) 2 Byte]
 * [Frame Sequence Number 1 Byte]
 * [CRC8 of Bytes before 1 Byte]
 * [Message Type (MSB second, LSB first) 2 Byte]
 * }
 * Frame Body{
 * [Frame Data]
 * [CRC16 of entire frame (head and frame) 2 Byte]
 * }
 * 
 */
namespace aruwlib
{

namespace serial
{

class DJISerial
{
 private:
    static const int16_t SERIAL_RX_BUFF_SIZE = 256;
    static const int16_t SERIAL_TX_BUFF_SIZE = 256;
    static const int16_t SERIAL_HEAD_BYTE = 0xA5;
    static const uint8_t FRAME_DATA_LENGTH_OFFSET = 1;
    static const uint8_t FRAME_SEQUENCENUM_OFFSET = 3;
    static const uint8_t FRAME_CRC8_OFFSET = 4;
    static const uint8_t FRAME_HEADER_LENGTH = 7;
    static const uint8_t FRAME_TYPE_OFFSET = 5;
    static const uint8_t FRAME_CRC16_LENGTH = 2;

 public:
    typedef enum
    {
        // PORT_UART1 = 0,
        PORT_UART2 = 1,
        PORT_UART6 = 2,
    } SerialPort;

    typedef struct
    {
        uint8_t headByte;
        uint16_t length;
        uint16_t type;
        uint8_t data[SERIAL_RX_BUFF_SIZE];
        modm::Timestamp messageTimestamp;
        uint8_t sequenceNumber;
    } SerialMessage_t;

    /**
     * Construct a Serial object
     * @param port serial port to work on
     * @param isRxCRCEnforcementEnabled if to enable Rx CRC Enforcement
     */
    DJISerial(
        SerialPort port,
        bool isRxCRCEnforcementEnabled
    );

    /**
     * Initialize serial
     */
    void initialize();

    /**
     * Send a Message. This constructs a message from the txMessage
     * @return true if succeed, false if failed
     */
    bool send(void);

    /**
     * Receive messages. Call periodically in order to not miss any messages
     * Tested with a delay of 10 microseconds with referee system
     */
    void updateSerial(void);

    /**
     * Called when a complete message is received, implemenent this yourself
     */
    virtual void messageReceiveCallback(SerialMessage_t completeMessage) = 0;

 private:
    enum SerialRxState
    {
        SERIAL_HEADER_SEARCH,
        PROCESS_FRAME_HEADER,
        PROCESS_FRAME_DATA
    };

    // serial port you are connected to
    SerialPort port;

    // stuff for rx, buffers to store parts of the header, state machine
    SerialRxState djiSerialRxState;
    SerialMessage_t newMessage;  // message in middle of being constructed
    SerialMessage_t mostRecentMessage;  // most recent complete message
    uint16_t frameCurrReadByte;
    uint8_t frameHeader[FRAME_HEADER_LENGTH];
    // handle electrical noise
    bool rxCRCEnforcementEnabled;

    // tx buffer
    uint8_t txBuffer[SERIAL_TX_BUFF_SIZE];

    bool verifyCRC16(uint8_t *message, uint32_t messageLength, uint16_t expectedCRC16);

    bool verifyCRC8(uint8_t *message, uint32_t messageLength, uint8_t expectedCRC8);

    uint32_t read(uint8_t *data, uint16_t length);

    uint32_t write(const uint8_t *data, uint16_t length);

 protected:  // subclasses can access the message that this class sends as to allow
             // for modification
    SerialMessage_t txMessage;
};

}  // namespace serial

}  // namespace aruwlib

#endif
