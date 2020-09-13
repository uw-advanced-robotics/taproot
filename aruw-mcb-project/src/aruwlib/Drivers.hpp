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

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#ifndef ENV_SIMULATOR
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
#include "control/command_mapper.hpp"
#include "control/command_scheduler.hpp"
#include "control/control_operator_interface.hpp"
#include "errors/error_controller.hpp"
#include "motor/dji_motor_tx_handler.hpp"
#else
#include <gmock/gmock.h>

#include "aruwlib/mock/AnalogMock.hpp"
#include "aruwlib/mock/CanMock.hpp"
#include "aruwlib/mock/CanRxHandlerMock.hpp"
#include "aruwlib/mock/CommandMapperMock.hpp"
#include "aruwlib/mock/CommandSchedulerMock.hpp"
#include "aruwlib/mock/ControlOperatorInterfaceMock.hpp"
#include "aruwlib/mock/DigitalMock.hpp"
#include "aruwlib/mock/DjiMotorTxHandlerMock.hpp"
#include "aruwlib/mock/ErrorControllerMock.hpp"
#include "aruwlib/mock/LedsMock.hpp"
#include "aruwlib/mock/Mpu6500Mock.hpp"
#include "aruwlib/mock/PwmMock.hpp"
#include "aruwlib/mock/RefSerialMock.hpp"
#include "aruwlib/mock/RemoteMock.hpp"
#include "aruwlib/mock/UartMock.hpp"
#include "aruwlib/mock/XavierSerialMock.hpp"
#endif

namespace aruwlib
{
class Drivers
{
    friend class DriversSingleton;

#ifdef ENV_SIMULATOR
public:
#endif
    Drivers()
        : can(),
          canRxHandler(this),
          analog(),
          digital(),
          leds(),
          pwm(),
          remote(this),
          mpu6500(this),
          uart(),
          xavierSerial(this),
          refSerial(this),
          commandScheduler(this),
          controlOperatorInterface(this),
          commandMapper(this),
          errorController(this),
          djiMotorTxHandler(this)
    {
    }

#ifndef ENV_SIMULATOR
public:
    can::Can can;
    can::CanRxHandler canRxHandler;
    gpio::Analog analog;
    gpio::Digital digital;
    gpio::Leds leds;
    gpio::Pwm pwm;
    Remote remote;
    sensors::Mpu6500 mpu6500;
    serial::Uart uart;
    serial::XavierSerial xavierSerial;
    serial::RefSerial refSerial;
    control::CommandScheduler commandScheduler;
    control::ControlOperatorInterface controlOperatorInterface;
    control::CommandMapper commandMapper;
    errors::ErrorController errorController;
    motor::DjiMotorTxHandler djiMotorTxHandler;
#else
    mock::CanMock can;
    mock::CanRxHandlerMock canRxHandler;
    mock::AnalogMock analog;
    mock::DigitalMock digital;
    mock::LedsMock leds;
    mock::PwmMock pwm;
    mock::RemoteMock remote;
    mock::Mpu6500Mock mpu6500;
    mock::UartMock uart;
    mock::XavierSerialMock xavierSerial;
    mock::RefSerialMock refSerial;
    mock::CommandSchedulerMock commandScheduler;
    mock::ControlOperatorInterfaceMock controlOperatorInterface;
    mock::CommandMapperMock commandMapper;
    mock::ErrorControllerMock errorController;
    mock::DjiMotorTxHandlerMock djiMotorTxHandler;
#endif
};  // class Drivers

}  // namespace aruwlib

#endif  // DRIVERS_HPP_
