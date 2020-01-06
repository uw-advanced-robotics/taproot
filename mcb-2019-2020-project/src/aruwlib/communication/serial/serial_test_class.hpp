#ifndef __SERIAL_TEST_CLASS_HPP__
#define __SERIAL_TEST_CLASS_HPP__

#include "dji_serial.hpp"

namespace aruwlib
{

namespace serial
{

class SerialTestClass : public DJISerial
{
 private:
    uint8_t messageId;

    uint8_t i;

 public:
    SerialTestClass();

    void messageReceiveCallback(SerialMessage_t completeMessage) override;

    void sendMessage(void);
};

}  // namespace serial

}  // namespace aruwlib

#endif
