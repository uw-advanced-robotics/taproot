#ifndef __SERIAL_TEST_CLASS_HPP__
#define __SERIAL_TEST_CLASS_HPP__

#include "dji_serial.hpp"

namespace aruwlib
{
namespace serial
{
/**
 * A simple serial tester to insure `DjiSerial` is working properly.
 *
 * @note to test with a single MCB, instantiate a test class and connect
 *      TX to RX. You should be able to check if messages are being received.
 *      Additionally, watch `i` to check if you are dropping messages.
 */
class SerialTestClass : public DJISerial
{
public:
    ///< Attaches this test class to `Uart2`.
    SerialTestClass();

    ///< Stores the sequenceNumber in `messageId`.
    void messageReceiveCallback(const SerialMessage& completeMessage) override;

    ///< Sends a message of length 1, the byte `60`, with the `sequenceNumber` incremented.
    void sendMessage();

private:
    uint8_t messageId;

    uint8_t i;
};

}  // namespace serial

}  // namespace aruwlib

#endif
