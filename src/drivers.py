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

def nice_mock(mock_name):
    return f"testing::NiceMock<{mock_name}>"

DRIVERS_AND_MODULE_DEPENDENCIES = [
    {
        "object-name": "arch::Profiler",
        "mock-object-name": "arch::Profiler",
        "object-instance-name": "profiler",
        "src-file": "tap/architecture/profiler.hpp",
        "mock-file": "tap/architecture/profiler.hpp",
        "constructor": "",
        "module-dependencies": [":communication:gpio:analog"],
    },
    {
        "object-name": "gpio::Analog",
        "mock-object-name": nice_mock("mock::AnalogMock"),
        "object-instance-name": "analog",
        "src-file": "tap/communication/gpio/analog.hpp",
        "mock-file": "tap/mock/analog_mock.hpp",
        "constructor": "",
        "module-dependencies": "",
    },
    {
        "object-name": "can::Can",
        "mock-object-name": nice_mock("mock::CanMock"),
        "object-instance-name": "can",
        "src-file": "tap/communication/can/can.hpp",
        "mock-file": "tap/mock/can_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:can"],
    },
    {
        "object-name": "can::CanRxHandler",
        "mock-object-name": nice_mock("mock::CanRxHandlerMock"),
        "object-instance-name": "canRxHandler",
        "src-file": "tap/communication/can/can_rx_handler.hpp",
        "mock-file": "tap/mock/can_rx_handler_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:can"],
    },
    {
        "object-name": "gpio::Digital",
        "mock-object-name": nice_mock("mock::DigitalMock"),
        "object-instance-name": "digital",
        "src-file": "tap/communication/gpio/digital.hpp",
        "mock-file": "tap/mock/digital_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:gpio:digital"],
    },
    {
        "object-name": "gpio::Leds",
        "mock-object-name": nice_mock("mock::LedMock"),
        "object-instance-name": "leds",
        "src-file": "tap/communication/gpio/leds.hpp",
        "mock-file": "tap/mock/leds_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:gpio:leds"],
    },
    {
        "object-name": "gpio::Pwm",
        "mock-object-name": nice_mock("mock::PwmMock"),
        "object-instance-name": "pwm",
        "src-file": "tap/communication/gpio/pwm.hpp",
        "mock-file": "tap/mock/pwm_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:gpio:pwm"],
    },
    {
        "object-name": "sensors::Mpu6500",
        "mock-object-name": nice_mock("mock::Mpu6500Mock"),
        "object-instance-name": "mpu6500",
        "src-file": "tap/communication/sensors/mpu6500/mpu6500.hpp",
        "mock-file": "tap/mock/mpu6500_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:sensors:mpu6500"],
    },
    {
        "object-name": "serial::RefSerial",
        "mock-object-name": nice_mock("mock::RefSerial"),
        "object-instance-name": "refSerial",
        "src-file": "tap/communication/serial/ref_serial.hpp",
        "mock-file": "tap/mock/ref_serial_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:serial:ref_serial"],
    },
    {
        "object-name": "Remote",
        "mock-object-name": nice_mock("mock::Remote"),
        "object-instance-name": "remote",
        "src-file": "tap/communication/serial/remote.hpp",
        "mock-file": "tap/mock/remote_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:serial:remote"],
    },
    {
        "object-name": "serial::Uart",
        "mock-object-name": nice_mock("mock::Uart"),
        "object-instance-name": "uart",
        "src-file": "tap/communication/serial/uart.hpp",
        "mock-file": "tap/mock/uart_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:serial"],
    },
    {
        "object-name": "communication::serial::TerminalSerial",
        "mock-object-name": nice_mock("mock::TerminalSerialMock"),
        "object-instance-name": "terminalSerial",
        "src-file": "tap/communication/serial/terminal_serial.hpp",
        "mock-file": "tap/mock/terminal_serial_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "control::CommandMapper",
        "mock-object-name": nice_mock("mock::CommandMapperMock"),
        "object-instance-name": "commandMapper",
        "src-file": "tap/control/command_mapper.hpp",
        "mock-file": "tap/mock/command_mapper_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "control::ControlOperatorInterface",
        "mock-object-name": nice_mock("mock::ControlOperatorInterfaceMock"),
        "object-instance-name": "controlOperatorInterface",
        "src-file": "tap/control/control_operator_interface.hpp",
        "mock-file": "tap/mock/control_operator_interface_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "control::SchedulerTerminalHandler",
        "mock-object-name": nice_mock("mock::SchedulerTerminalHandlerMock"),
        "object-instance-name": "schedulerTerminalHandler",
        "src-file": "tap/control/scheduler_terminal_handler.hpp",
        "mock-file": "tap/mock/scheduler_terminal_handler_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "errors::ErrorController",
        "mock-object-name": nice_mock("mock::ErrorControllerMock"),
        "object-instance-name": "errorController",
        "src-file": "tap/errors/error_controller.hpp",
        "mock-file": "tap/mock/error_controller_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "motor::DjiMotorTerminalSerialHandler",
        "mock-object-name": nice_mock("mock::DjiMotorTerminalSerialHandler"),
        "object-instance-name": "djiMotorTerminalSerialHandler",
        "src-file": "tap/motor/dji_motor_terminal_serial_handler.hpp",
        "mock-file": "tap/mock/dji_motor_terminal_serial_handler_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:serial:terminal_serial"],
    },
    {
        "object-name": "motor::DjiMotorTxHandler",
        "mock-object-name": nice_mock("mock::DjiMotorTxHandlerMock"),
        "object-instance-name": "djiMotorTxHandler",
        "src-file": "tap/motor/dji_motor_tx_handler.hpp",
        "mock-file": "tap/mock/dji_motor_tx_handler_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
]

def should_driver_be_generated(env, driver):
    return all(env.has_module(dependency) for dependency in driver["module-dependencies"])

def getNameSorted(env, name):
    return sorted([driver[name] for driver in DRIVERS_AND_MODULE_DEPENDENCIES if should_driver_be_generated(env, driver)])

def getSrcFilesSorted(env):
    return getNameSorted(env, "src-file")

def getMockFilesSorted(env):
    return getNameSorted(env, "mock-file")

def getObjectAndMockNames(env):
    return [{"object-name": driver["object-name"],
             "mock-object-name": driver["mock-object-name"],
             "object-instance-name": driver["object-instance-name"],
             "constructor": driver["constructor"]} for driver in DRIVERS_AND_MODULE_DEPENDENCIES if should_driver_be_generated(env, driver)]
