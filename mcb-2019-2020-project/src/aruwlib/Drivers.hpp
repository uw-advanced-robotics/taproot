#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#include "communication/can/can.hpp"
#include "communication/can/can_rx_handler.hpp"
#include "communication/gpio/analog.hpp"
#include "communication/gpio/digital.hpp"
#include "communication/gpio/leds.hpp"
#include "communication/gpio/pwm.hpp"
#include "communication/remote.hpp"
#include "communication/sensors/mpu6500/mpu6500.hpp"
#include "communication/serial/ref_serial.hpp"
#include "communication/serial/uart.hpp"
#include "communication/serial/xavier_serial.hpp"
#include "control/command_scheduler.hpp"
#include "control/control_operator_interface.hpp"
#include "control/controller_mapper.hpp"
#include "errors/error_controller.hpp"
#include "motor/dji_motor_tx_handler.hpp"

namespace aruwlib
{
class Drivers
{
public:
    static can::Can can;
    static can::CanRxHandler canRxHandler;
    static gpio::Analog analog;
    static gpio::Digital digital;
    static gpio::Leds leds;
    static gpio::Pwm pwm;
    static Remote remote;
    static sensors::Mpu6500 mpu6500;
    static serial::Uart uart;
    static serial::XavierSerial xavierSerial;
    static serial::RefSerial refSerial;
    static control::CommandScheduler commandScheduler;
    static control::ControlOperatorInterface controlOperatorInterface;
    static control::IoMapper ioMapper;
    static errors::ErrorController errorController;
    static motor::DjiMotorTxHandler djiMotorTxHandler;

#ifdef ENV_SIMULATOR
    static void reset();

private:
    static void resetCan();
    static void resetCanRxHandler();
    static void resetAnalog();
    static void resetDigital();
    static void resetLeds();
    static void resetPwm();
    static void resetRemote();
    static void resetMpu6500();
    static void resetUart();
    static void resetXavierSerial();
    static void resetRefSerial();
    static void resetCommandScheduler();
    static void resetControlOperatorInterface();
    static void resetIoMapper();
    static void resetErrorController();
    static void resetDjiMotorTxHandler();
#endif
};  // class Drivers
}  // namespace aruwlib

#endif  // DRIVERS_HPP_
