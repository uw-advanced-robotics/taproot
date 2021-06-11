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

#include "AnalogMock.hpp"
#include "CanMock.hpp"
#include "CanRxHandlerMock.hpp"
#include "CanRxListenerMock.hpp"
#include "CommandMapperMock.hpp"
#include "CommandMock.hpp"
#include "CommandSchedulerMock.hpp"
#include "ControlOperatorInterfaceMock.hpp"
#include "DJIMotorMock.hpp"
#include "DigitalMock.hpp"
#include "DjiMotorTerminalSerialHandlerMock.hpp"
#include "DjiMotorTxHandlerMock.hpp"
#include "ErrorControllerMock.hpp"
#include "LedsMock.hpp"
#include "Mpu6500Mock.hpp"
#include "OledDisplayMock.hpp"
#include "PwmMock.hpp"
#include "RefSerialMock.hpp"
#include "RemoteMock.hpp"
#include "SchedulerTerminalHandlerMock.hpp"
#include "SubsystemMock.hpp"
#include "TerminalSerialMock.hpp"
#include "UartMock.hpp"
#include "XavierSerialMock.hpp"

// A file for listing all mock constructors and destructors since doing
// so in a source file allows for faster compilation than defining constructors
// in the headers
namespace aruwlib::mock
{
AnalogMock::AnalogMock() {}
AnalogMock::~AnalogMock() {}

CanMock::CanMock() {}
CanMock::~CanMock() {}

CanRxListenerMock::CanRxListenerMock(aruwlib::Drivers *drivers, uint32_t id, can::CanBus bus)
    : can::CanRxListener(drivers, id, bus)
{
}
CanRxListenerMock::~CanRxListenerMock() {}

CanRxHandlerMock::CanRxHandlerMock(aruwlib::Drivers *drivers) : can::CanRxHandler(drivers) {}
CanRxHandlerMock::~CanRxHandlerMock() {}

CommandMock::CommandMock() : Command()
{
    // Most of the time tests expect that we are adding commands that
    // are ready to be added. This makes tests cleaner
    ON_CALL(*this, isReady).WillByDefault(testing::Return(true));
}
CommandMock::~CommandMock() {}

CommandMapperMock::CommandMapperMock(aruwlib::Drivers *drivers) : control::CommandMapper(drivers) {}
CommandMapperMock::~CommandMapperMock() {}

ControlOperatorInterfaceMock::ControlOperatorInterfaceMock(aruwlib::Drivers *drivers)
    : aruwlib::control::ControlOperatorInterface(drivers)
{
}
ControlOperatorInterfaceMock::~ControlOperatorInterfaceMock() {}

CommandSchedulerMock::CommandSchedulerMock(aruwlib::Drivers *drivers)
    : control::CommandScheduler(drivers)
{
}
CommandSchedulerMock::~CommandSchedulerMock() {}

DjiMotorMock::DjiMotorMock(
    Drivers *drivers,
    aruwlib::motor::MotorId desMotorIdentifier,
    aruwlib::can::CanBus motorCanBus,
    bool isInverted,
    const char *name)
    : DjiMotor(drivers, desMotorIdentifier, motorCanBus, isInverted, name)
{
}
DjiMotorMock::~DjiMotorMock() {}

DigitalMock::DigitalMock() {}
DigitalMock::~DigitalMock() {}

DjiMotorTxHandlerMock::DjiMotorTxHandlerMock(aruwlib::Drivers *drivers)
    : aruwlib::motor::DjiMotorTxHandler(drivers)
{
}
DjiMotorTxHandlerMock::~DjiMotorTxHandlerMock() {}

DjiMotorTerminalSerialHandlerMock::DjiMotorTerminalSerialHandlerMock(aruwlib::Drivers *drivers)
    : motor::DjiMotorTerminalSerialHandler(drivers)
{
}
DjiMotorTerminalSerialHandlerMock::~DjiMotorTerminalSerialHandlerMock() {}

LedsMock::LedsMock() {}
LedsMock::~LedsMock() {}

ErrorControllerMock::ErrorControllerMock(aruwlib::Drivers *drivers)
    : aruwlib::errors::ErrorController(drivers)
{
}
ErrorControllerMock::~ErrorControllerMock() {}

Mpu6500Mock::Mpu6500Mock(aruwlib::Drivers *drivers) : aruwlib::sensors::Mpu6500(drivers) {}
Mpu6500Mock::~Mpu6500Mock() {}

OledDisplayMock::OledDisplayMock(Drivers *drivers) : display::OledDisplay(drivers) {}
OledDisplayMock::~OledDisplayMock() {}

PwmMock::PwmMock() {}
PwmMock::~PwmMock() {}

RefSerialMock::RefSerialMock(Drivers *drivers) : serial::RefSerial(drivers) {}
RefSerialMock::~RefSerialMock() {}

RemoteMock::RemoteMock(aruwlib::Drivers *drivers) : aruwlib::Remote(drivers) {}
RemoteMock::~RemoteMock() {}

SchedulerTerminalHandlerMock::SchedulerTerminalHandlerMock(Drivers *drivers)
    : control::SchedulerTerminalHandler(drivers)
{
}
SchedulerTerminalHandlerMock::~SchedulerTerminalHandlerMock() {}

SubsystemMock::SubsystemMock(Drivers *drivers) : control::Subsystem(drivers) {}
SubsystemMock::~SubsystemMock() {}

TerminalSerialMock::TerminalSerialMock(Drivers *drivers)
    : communication::serial::TerminalSerial(drivers)
{
}
TerminalSerialMock::~TerminalSerialMock() {}

UartMock::UartMock() {}
UartMock::~UartMock() {}

XavierSerialMock::XavierSerialMock(Drivers *drivers) : serial::XavierSerial(drivers) {}
XavierSerialMock::~XavierSerialMock() {}
}  // namespace aruwlib::mock
