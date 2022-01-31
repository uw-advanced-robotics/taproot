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

def strict_mock(mock_name):
    return f"testing::StrictMock<{mock_name}>"

"""
A dictionary storing data about each driver that has
been defined in code. This data is used to construct
a drivers object. The module-dependencies specified
will determine if the driver defined below will be
added to the main drivers object.
"""
DRIVERS_AND_MODULE_DEPENDENCIES = [
    {
        "object-name": "arch::Profiler",
        "mock-object-name": "arch::Profiler",
        "src-file": "tap/architecture/profiler.hpp",
        "mock-header": "tap/architecture/profiler.hpp",
        "constructor": "",
        "module-dependencies": [":communication:gpio:analog"],
    },
    {
        "object-name": "gpio::Analog",
        "mock-object-name": nice_mock("mock::AnalogMock"),
        "src-file": "tap/communication/gpio/analog.hpp",
        "mock-header": "tap/mock/analog_mock.hpp",
        "constructor": "",
        "module-dependencies": "",
    },
    {
        "object-name": "can::Can",
        "mock-object-name": nice_mock("mock::CanMock"),
        "src-file": "tap/communication/can/can.hpp",
        "mock-header": "tap/mock/can_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:can"],
    },
    {
        "object-name": "can::CanRxHandler",
        "mock-object-name": nice_mock("mock::CanRxHandlerMock"),
        "src-file": "tap/communication/can/can_rx_handler.hpp",
        "mock-header": "tap/mock/can_rx_handler_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:can"],
    },
    {
        "object-name": "gpio::Digital",
        "mock-object-name": nice_mock("mock::DigitalMock"),
        "src-file": "tap/communication/gpio/digital.hpp",
        "mock-header": "tap/mock/digital_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:gpio:digital"],
    },
    {
        "object-name": "gpio::Leds",
        "mock-object-name": nice_mock("mock::LedsMock"),
        "src-file": "tap/communication/gpio/leds.hpp",
        "mock-header": "tap/mock/leds_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:gpio:leds"],
    },
    {
        "object-name": "gpio::Pwm",
        "mock-object-name": nice_mock("mock::PwmMock"),
        "src-file": "tap/communication/gpio/pwm.hpp",
        "mock-header": "tap/mock/pwm_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:gpio:pwm"],
    },
    {
        "object-name": "sensors::Mpu6500",
        "mock-object-name": nice_mock("mock::Mpu6500Mock"),
        "src-file": "tap/communication/sensors/mpu6500/mpu6500.hpp",
        "mock-header": "tap/mock/mpu6500_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:sensors:mpu6500"],
    },
    {
        "object-name": "serial::RefSerial",
        "mock-object-name": nice_mock("mock::RefSerialMock"),
        "src-file": "tap/communication/serial/ref_serial.hpp",
        "mock-header": "tap/mock/ref_serial_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:serial:ref_serial"],
    },
    {
        "object-name": "Remote",
        "mock-object-name": nice_mock("mock::RemoteMock"),
        "src-file": "tap/communication/serial/remote.hpp",
        "mock-header": "tap/mock/remote_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:serial:remote"],
    },
    {
        "object-name": "serial::Uart",
        "mock-object-name": nice_mock("mock::UartMock"),
        "src-file": "tap/communication/serial/uart.hpp",
        "mock-header": "tap/mock/uart_mock.hpp",
        "constructor": "",
        "module-dependencies": [":communication:serial"],
    },
    {
        "object-name": "communication::serial::TerminalSerial",
        "mock-object-name": nice_mock("mock::TerminalSerialMock"),
        "src-file": "tap/communication/serial/terminal_serial.hpp",
        "mock-header": "tap/mock/terminal_serial_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "control::CommandMapper",
        "mock-object-name": nice_mock("mock::CommandMapperMock"),
        "src-file": "tap/control/command_mapper.hpp",
        "mock-header": "tap/mock/command_mapper_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "control::SchedulerTerminalHandler",
        "mock-object-name": nice_mock("mock::SchedulerTerminalHandlerMock"),
        "src-file": "tap/control/scheduler_terminal_handler.hpp",
        "mock-header": "tap/mock/scheduler_terminal_handler_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "errors::ErrorController",
        "mock-object-name": strict_mock("mock::ErrorControllerMock"),
        "src-file": "tap/errors/error_controller.hpp",
        "mock-header": "tap/mock/error_controller_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "motor::DjiMotorTerminalSerialHandler",
        "mock-object-name": nice_mock("mock::DjiMotorTerminalSerialHandlerMock"),
        "src-file": "tap/motor/dji_motor_terminal_serial_handler.hpp",
        "mock-header": "tap/mock/dji_motor_terminal_serial_handler_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:serial:terminal_serial"],
    },
    {
        "object-name": "motor::DjiMotorTxHandler",
        "mock-object-name": nice_mock("mock::DjiMotorTxHandlerMock"),
        "src-file": "tap/motor/dji_motor_tx_handler.hpp",
        "mock-header": "tap/mock/dji_motor_tx_handler_mock.hpp",
        "constructor": "this",
        "module-dependencies": "",
    },
    {
        "object-name": "sensors::Mpu6500TerminalSerialHandler",
        "mock-object-name": nice_mock("mock::Mpu6500TerminalSerialHandlerMock"),
        "src-file": "tap/communication/sensors/mpu6500/mpu6500_terminal_serial_handler.hpp",
        "mock-header": "tap/mock/mpu6500_terminal_serial_handler_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:serial:terminal_serial", ":communication:sensors:mpu6500"],
    },
    {
        "object-name": "communication::sensors::bmi088::Bmi088",
        "mock-object-name": nice_mock("mock::Bmi088Mock"),
        "src-file": "tap/communication/sensors/bmi088/bmi088.hpp",
        "mock-header": "tap/mock/bmi088_mock.hpp",
        "constructor": "this",
        "module-dependencies": [":communication:sensors:bmi088"],
    }
]

def should_driver_be_generated(env, driver):
    return all(env.has_module(dependency) for dependency in driver["module-dependencies"])

def get_names_sorted(env, name):
    return sorted([driver[name] for driver in DRIVERS_AND_MODULE_DEPENDENCIES if should_driver_be_generated(env, driver)])

def get_src_files_sorted(env):
    return get_names_sorted(env, "src-file")

def get_mock_headers_sorted(env):
    return get_names_sorted(env, "mock-header")

def get_object_and_mock_names(env):
    objects_and_mocks = []
    for driver in DRIVERS_AND_MODULE_DEPENDENCIES:
        if should_driver_be_generated(env, driver):
            object_instance_name_pascal = driver["object-name"].split("::")[-1]
            object_instance_name_camel = object_instance_name_pascal[0].lower() + object_instance_name_pascal[1:]
            objects_and_mocks.append({
                "object-name": driver["object-name"],
                "mock-object-name": driver["mock-object-name"],
                "object-instance-name": object_instance_name_camel,
                "constructor": driver["constructor"]
            })
    return objects_and_mocks
