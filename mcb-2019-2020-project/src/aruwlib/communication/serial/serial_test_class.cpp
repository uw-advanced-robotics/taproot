#include "serial_test_class.hpp"

namespace aruwlib
{

namespace serial
{

SerialTestClass::SerialTestClass():
DJISerial(Uart::UartPort::Uart2, true),
messageId(0), i(0)
{}

void SerialTestClass::messageReceiveCallback(SerialMessage completeMessage)
{
    messageId = completeMessage.sequenceNumber;
}

void SerialTestClass::sendMessage()
{
    this->txMessage.length = 1;
    this->txMessage.headByte = 0xa5;
    this->txMessage.sequenceNumber = i;
    this->txMessage.type = 4;
    this->txMessage.data[0] = 60;
    this->send();
    i++;
}

}  // namespace serial

}  // namespace aruwlib
