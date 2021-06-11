/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/AnalogMock.hpp"
#include "aruwlib/mock/CanMock.hpp"
#include "aruwlib/mock/CanRxHandlerMock.hpp"
#include "aruwlib/mock/CommandMapperMock.hpp"
#include "aruwlib/mock/CommandSchedulerMock.hpp"
#include "aruwlib/mock/ControlOperatorInterfaceMock.hpp"
#include "aruwlib/mock/DigitalMock.hpp"
#include "aruwlib/mock/DjiMotorTerminalSerialHandlerMock.hpp"
#include "aruwlib/mock/DjiMotorTxHandlerMock.hpp"
#include "aruwlib/mock/ErrorControllerMock.hpp"
#include "aruwlib/mock/LedsMock.hpp"
#include "aruwlib/mock/Mpu6500Mock.hpp"
#include "aruwlib/mock/OledDisplayMock.hpp"
#include "aruwlib/mock/PwmMock.hpp"
#include "aruwlib/mock/RefSerialMock.hpp"
#include "aruwlib/mock/RemoteMock.hpp"
#include "aruwlib/mock/SchedulerTerminalHandlerMock.hpp"
#include "aruwlib/mock/TerminalSerialMock.hpp"
#include "aruwlib/mock/UartMock.hpp"
#include "aruwlib/mock/XavierSerialMock.hpp"

#include "architecture/profiler.hpp"
#else
#include "architecture/profiler.hpp"
#include "communication/can/can.hpp"
#include "communication/can/can_rx_handler.hpp"
#include "communication/gpio/analog.hpp"
#include "communication/gpio/digital.hpp"
#include "communication/gpio/leds.hpp"
#include "communication/gpio/pwm.hpp"
#include "communication/remote.hpp"
#include "communication/sensors/mpu6500/mpu6500.hpp"
#include "communication/serial/TerminalSerial.hpp"
#include "communication/serial/ref_serial.hpp"
#include "communication/serial/uart.hpp"
#include "communication/serial/xavier_serial.hpp"
#include "control/CommandMapper.hpp"
#include "control/ControlOperatorInterface.hpp"
#include "control/SchedulerTerminalHandler.hpp"
#include "control/command_scheduler.hpp"
#include "display/OledDisplay.hpp"
#include "errors/error_controller.hpp"
#include "motor/DjiMotorTerminalSerialHandler.hpp"
#include "motor/dji_motor_tx_handler.hpp"
#endif

namespace aruwlib
{
class Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
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
          refSerial(this),
#ifdef ENV_UNIT_TESTS
          commandScheduler(this),
#else
          commandScheduler(this, true),
#endif
          controlOperatorInterface(this),
          xavierSerial(this),
          commandMapper(this),
          errorController(this),
          terminalSerial(this),
          djiMotorTxHandler(this),
          oledDisplay(this),
          profiler(),
          djiMotorTerminalSerialHandler(this),
          schedulerTerminalHandler(this)
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::CanMock> can;
    testing::NiceMock<mock::CanRxHandlerMock> canRxHandler;
    testing::NiceMock<mock::AnalogMock> analog;
    testing::NiceMock<mock::DigitalMock> digital;
    testing::NiceMock<mock::LedsMock> leds;
    testing::NiceMock<mock::PwmMock> pwm;
    testing::NiceMock<mock::RemoteMock> remote;
    testing::NiceMock<mock::Mpu6500Mock> mpu6500;
    testing::NiceMock<mock::UartMock> uart;
    testing::NiceMock<mock::RefSerialMock> refSerial;
    testing::NiceMock<mock::CommandSchedulerMock> commandScheduler;
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::XavierSerialMock> xavierSerial;
    testing::NiceMock<mock::CommandMapperMock> commandMapper;
    mock::ErrorControllerMock errorController;
    testing::NiceMock<mock::TerminalSerialMock> terminalSerial;
    testing::NiceMock<mock::DjiMotorTxHandlerMock> djiMotorTxHandler;
    testing::NiceMock<mock::OledDisplayMock> oledDisplay;
    arch::Profiler profiler;
    testing::NiceMock<mock::DjiMotorTerminalSerialHandlerMock> djiMotorTerminalSerialHandler;
    testing::NiceMock<mock::SchedulerTerminalHandlerMock> schedulerTerminalHandler;
#else
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
    serial::RefSerial refSerial;
    control::CommandScheduler commandScheduler;
    control::ControlOperatorInterface controlOperatorInterface;
    serial::XavierSerial xavierSerial;
    control::CommandMapper commandMapper;
    errors::ErrorController errorController;
    communication::serial::TerminalSerial terminalSerial;
    motor::DjiMotorTxHandler djiMotorTxHandler;
    display::OledDisplay oledDisplay;
    arch::Profiler profiler;
    motor::DjiMotorTerminalSerialHandler djiMotorTerminalSerialHandler;
    control::SchedulerTerminalHandler schedulerTerminalHandler;
#endif
};  // class Drivers

}  // namespace aruwlib

#endif  // DRIVERS_HPP_
