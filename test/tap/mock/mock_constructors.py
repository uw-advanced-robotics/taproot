# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of Taproot.
#
# Taproot is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Taproot is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Taproot.  If not, see <https://www.gnu.org/licenses/>.

import textwrap

"""
A dictionary pairing mock filenames with C++ code snippets
that define the constructor and destructor for the associated
mock. These constructors and destructors are used to generate
mock_constructors.cpp
"""
MOCK_CONSTRUCTORS_DESTRUCTORS = [
    {
        "mock-file": "analog_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            AnalogMock::AnalogMock() {}
            AnalogMock::~AnalogMock() {}""")
    },
    {
        "mock-file": "can_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            CanMock::CanMock() {}
            CanMock::~CanMock() {}""")
    },
    {
        "mock-file": "can_rx_listener_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            CanRxListenerMock::CanRxListenerMock(tap::Drivers *drivers, uint32_t id, can::CanBus bus)
                : can::CanRxListener(drivers, id, bus)
            {
            }
            CanRxListenerMock::~CanRxListenerMock() {}""")
    },
    {
        "mock-file": "can_rx_handler_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            CanRxHandlerMock::CanRxHandlerMock(tap::Drivers *drivers) : can::CanRxHandler(drivers) {}
            CanRxHandlerMock::~CanRxHandlerMock() {}""")
    },
    {
        "mock-file": "command_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            CommandMock::CommandMock() : Command()
            {
                // Most of the time tests expect that we are adding commands that
                // are ready to be added. This makes tests cleaner
                ON_CALL(*this, isReady).WillByDefault(testing::Return(true));
            }
            CommandMock::~CommandMock() {}""")
    },
    {
        "mock-file": "command_mapper_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            CommandMapperMock::CommandMapperMock(tap::Drivers *drivers) : control::CommandMapper(drivers) {}
            CommandMapperMock::~CommandMapperMock() {}""")
    },
    {
        "mock-file": "control_operator_interface_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            ControlOperatorInterfaceMock::ControlOperatorInterfaceMock(tap::Drivers *drivers)
                : tap::control::ControlOperatorInterface(drivers)
            {
            }
            ControlOperatorInterfaceMock::~ControlOperatorInterfaceMock() {}""")
    },
    {
        "mock-file": "command_scheduler_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            CommandSchedulerMock::CommandSchedulerMock(tap::Drivers *drivers)
                : control::CommandScheduler(drivers)
            {
            }
            CommandSchedulerMock::~CommandSchedulerMock() {}""")
    },
    {
        "mock-file": "dji_motor_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            DjiMotorMock::DjiMotorMock(
                Drivers *drivers,
                tap::motor::MotorId desMotorIdentifier,
                tap::can::CanBus motorCanBus,
                bool isInverted,
                const char *name)
                : DjiMotor(drivers, desMotorIdentifier, motorCanBus, isInverted, name)
            {
            }
            DjiMotorMock::~DjiMotorMock() {}""")
    },
    {
        "mock-file": "digital_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            DigitalMock::DigitalMock() {}
            DigitalMock::~DigitalMock() {}""")
    },
    {
        "mock-file": "dji_motor_tx_handler_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            DjiMotorTxHandlerMock::DjiMotorTxHandlerMock(tap::Drivers *drivers)
                : tap::motor::DjiMotorTxHandler(drivers)
            {
            }
            DjiMotorTxHandlerMock::~DjiMotorTxHandlerMock() {}""")
    },
    {
        "mock-file": "dji_motor_terminal_serial_handler_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            DjiMotorTerminalSerialHandlerMock::DjiMotorTerminalSerialHandlerMock(tap::Drivers *drivers)
                : motor::DjiMotorTerminalSerialHandler(drivers)
            {
            }
            DjiMotorTerminalSerialHandlerMock::~DjiMotorTerminalSerialHandlerMock() {}""")
    },
    {
        "mock-file": "leds_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            LedsMock::LedsMock() {}
            LedsMock::~LedsMock() {}""")
    },
    {
        "mock-file": "error_controller_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            ErrorControllerMock::ErrorControllerMock(tap::Drivers *drivers)
                : tap::errors::ErrorController(drivers)
            {
            }
            ErrorControllerMock::~ErrorControllerMock() {}""")
    },
    {
        "mock-file": "mpu6500_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            Mpu6500Mock::Mpu6500Mock(tap::Drivers *drivers) : tap::sensors::Mpu6500(drivers) {}
            Mpu6500Mock::~Mpu6500Mock() {}""")
    },
    {
        "mock-file": "pwm_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            PwmMock::PwmMock() {}
            PwmMock::~PwmMock() {}""")
    },
    {
        "mock-file": "ref_serial_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            RefSerialMock::RefSerialMock(Drivers *drivers) : serial::RefSerial(drivers) {}
            RefSerialMock::~RefSerialMock() {}""")
    },
    {
        "mock-file": "remote_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            RemoteMock::RemoteMock(tap::Drivers *drivers) : tap::Remote(drivers) {}
            RemoteMock::~RemoteMock() {}""")
    },
    {
        "mock-file": "scheduler_terminal_handler_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            SchedulerTerminalHandlerMock::SchedulerTerminalHandlerMock(Drivers *drivers)
                : control::SchedulerTerminalHandler(drivers)
            {
            }
            SchedulerTerminalHandlerMock::~SchedulerTerminalHandlerMock() {}""")
    },
    {
        "mock-file": "subsystem_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            SubsystemMock::SubsystemMock(Drivers *drivers) : control::Subsystem(drivers) {}
            SubsystemMock::~SubsystemMock() {}""")
    },
    {
        "mock-file": "terminal_serial_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            TerminalSerialMock::TerminalSerialMock(Drivers *drivers)
                : communication::serial::TerminalSerial(drivers)
            {
            }
            TerminalSerialMock::~TerminalSerialMock() {}""")
    },
    {
        "mock-file": "uart_mock.hpp",
        "constructor-destructor":
            textwrap.dedent("""
            UartMock::UartMock() {}
            UartMock::~UartMock() {}""")
    },
]
