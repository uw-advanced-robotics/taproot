#include "Drivers.hpp"

namespace aruwlib
{
can::Can Drivers::can;
can::CanRxHandler Drivers::canRxHandler;
gpio::Analog Drivers::analog;
gpio::Digital Drivers::digital;
gpio::Leds Drivers::leds;
gpio::Pwm Drivers::pwm;
Remote Drivers::remote;
sensors::Mpu6500 Drivers::mpu6500;
serial::Uart Drivers::uart;
serial::XavierSerial Drivers::xavierSerial;
serial::RefSerial Drivers::refSerial;
control::CommandScheduler Drivers::commandScheduler;
control::ControlOperatorInterface Drivers::controlOperatorInterface;
control::IoMapper Drivers::ioMapper;
errors::ErrorController Drivers::errorController;
motor::DjiMotorTxHandler Drivers::djiMotorTxHandler;

#ifdef ENV_SIMULATOR
void Drivers::reset()
{
    resetCan();
    resetCanRxHandler();
    resetAnalog();
    resetDigital();
    resetLeds();
    resetPwm();
    resetRemote();
    resetMpu6500();
    resetUart();
    resetXavierSerial();
    resetRefSerial();
    resetCommandScheduler();
    resetControlOperatorInterface();
    resetIoMapper();
    resetErrorController();
    resetDjiMotorTxHandler();
}

void Drivers::resetCan() {}
void Drivers::resetCanRxHandler() {}
void Drivers::resetAnalog() {}
void Drivers::resetDigital() {}
void Drivers::resetLeds() {}
void Drivers::resetPwm() {}
void Drivers::resetRemote() {}
void Drivers::resetMpu6500() {}
void Drivers::resetUart() {}
void Drivers::resetXavierSerial() {}
void Drivers::resetRefSerial() {}
void Drivers::resetCommandScheduler() {}
void Drivers::resetControlOperatorInterface() {}
void Drivers::resetIoMapper() {}
void Drivers::resetErrorController() {}
void Drivers::resetDjiMotorTxHandler() {}
#endif
}  // namespace aruwlib
