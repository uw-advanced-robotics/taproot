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

import glob
import lxml

from lbuild_utils import repo_path_rel_repolb

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Union, Set, TypeVar, Generic
from enum import Enum
from abc import ABC, abstractmethod
from collections import namedtuple, defaultdict
from functools import lru_cache
from lbuild.exception import LbuildValidateException as ValidateException

parsed_board_info = {}

class Instance(ABC):
    raw_name: Union[str, int]
    alias: Optional[str]
    comment: Optional[str]

    def __init__(self, xml, comment):
        name = xml.get("name")
        self.raw_name = int(name) if name.isnumeric() else name
        
        self.alias = xml.get("alias")
        self.comment = comment.strip() if comment is not None else None

    def display_name(self):
        return self.alias if self.alias is not None else self.name()

    def __repr__(self):
        return self.display_name()

    @abstractmethod
    def get_used_pins(self) -> List[str]:
        ...

    def name(self) -> str:
        return str(self.raw_name)

    def __lt__(self, other):
        return self.raw_name < other.raw_name


class CanBus(Instance):
    rx: str
    tx: str

    def __init__(self, xml, comment):
        assert xml.tag == "can"
        super().__init__(xml, comment)

        self.rx = xml.get("rx")
        self.tx = xml.get("tx")

    def get_used_pins(self) -> List[str]:
        return [self.rx, self.tx]

    def name(self) -> str:
        return f"Can{self.raw_name}"


class Uart(Instance):
    usart: bool
    rx: Optional[str]
    tx: Optional[str]

    def __init__(self, xml, comment):
        assert xml.tag == "uart" or xml.tag == "usart"
        super().__init__(xml, comment)

        self.usart = xml.tag == "usart"
        self.rx = xml.get("rx", None)
        self.tx = xml.get("tx", None)

    def get_used_pins(self) -> List[str]:
        return [pin for pin in [self.rx, self.tx] if pin is not None]
    
    def name(self) -> str:
        return f"U{'s' if self.usart else ''}art{self.raw_name}"

adcs = {}
class Adc:
    raw_name: int
    pins: List["Gpio"]

    def __init__(self, raw_name):
        self.raw_name = raw_name
        self.pins = []

    def name(self) -> str:
        return f"Adc{self.raw_name}"

    def add_pin(self, pin):
        self.pins.append(pin)

    @staticmethod
    def get(raw_name):
        global adcs
        if raw_name not in adcs:
            adcs[raw_name] = Adc(raw_name)

        return adcs[raw_name]

    def __lt__(self, other):
        return self.raw_name < other.raw_name

    def name(self) -> str:
        return f"Adc{self.raw_name}"

timers = {}
class Timer:
    raw_name: int
    pins: List["Gpio"]

    def __init__(self, raw_name):
        self.raw_name = raw_name
        self.pins = []

    def name(self) -> str:
        return f"Timer{self.raw_name}"

    def add_pin(self, pin):
        self.pins.append(pin)

    @staticmethod
    def get(raw_name):
        global timers
        if raw_name not in timers:
            timers[raw_name] = Timer(raw_name)

        return timers[raw_name]

    def __lt__(self, other):
        return self.raw_name < other.raw_name

    def name(self) -> str:
        return f"Timer{self.raw_name}"

class Feature(ABC):
    pass

class AdcFeature(Feature):
    adc: int
    in_channel: str

    @staticmethod
    def parse(gpio, xml):
        assert xml.tag == "adc"

        raw_name = int(xml.get("name")[3:])
        in_channel = xml.get("in")

        feature = AdcFeature(raw_name, in_channel)
        feature.adc.add_pin(gpio)

        return feature

    def __init__(self, raw_name, in_channel):
        self.adc = Adc.get(raw_name)
        self.in_channel = in_channel

class TimerFeature(Feature):
    timer: Timer
    channel: str

    @staticmethod
    def parse(gpio, xml):
        assert xml.tag == "timer"

        raw_name = int(xml.get("name")[5:])
        channel = xml.get("channel")

        feature = TimerFeature(raw_name, channel)
        feature.timer.add_pin(gpio)

        return feature

    def __init__(self, raw_name, channel):
        self.timer = Timer.get(raw_name)
        self.channel = channel

# TODO: Add other features, such as interrupts.

class Gpio(Instance):
    gpio_type: str

    adc: AdcFeature
    timer: TimerFeature

    def __init__(self, xml, comment):
        assert xml.tag in  ["gpio", "out", "in", "pwm", "analog"]
        super().__init__(xml, comment)

        self.adc = None
        self.timer = None

        for child in xml.iterchildren():
            if child.tag == "adc":
                self.adc = AdcFeature.parse(self, child)
            elif child.tag == "timer":
                self.timer = TimerFeature.parse(self, child)

        self.gpio_type = xml.tag

        if self.gpio_type == "analog":
            self.adc = AdcFeature(int(xml.get("adc")[3:]), xml.get("in"))
            self.adc.adc.add_pin(self)
        elif self.gpio_type == "pwm":
            self.timer = TimerFeature(int(xml.get("timer")[5:]), xml.get("channel"))
            self.timer.timer.add_pin(self)

    def get_used_pins(self) -> List[str]:
        return [self.raw_name]


class GroupGpio(Gpio):
    group: Instance

    def __init__(self, xml, comment, group):
        super().__init__(xml, comment)
        self.group = group
    
    def display_name(self):
        return self.group.display_name() + super().display_name()


class GpioGroup(Instance):
    default: Optional[str]
    gpios: List[Gpio]

    def __init__(self, xml, comment):
        assert xml.tag == "gpio-group"
        super().__init__(xml, comment)

        self.alias = xml.get("alias")
        self.default = xml.get("default", None)

        self.gpios = [GroupGpio(child, None, self) for child in xml.iterchildren()]

    def get_used_pins(self) -> List[str]:   
        return [pin.raw_name for pin in self.gpios]


class CommunicationGroup(Instance):
    gpios: List[Gpio]

    def __init__(self, xml, comment):
        super().__init__(xml, comment)

        self.gpios = [GroupGpio(child, None, self) for child in xml.iterchildren()]

    def get_used_pins(self) -> List[str]:
        return [pin.name for pin in self.gpios if pin is not None]


class I2C(CommunicationGroup):
    sda: str
    scl: str

    def __init__(self, xml, comment):
        super().__init__(xml, comment)
        self.sda = xml.get("sda")
        self.scl = xml.get("scl")

    def get_used_pins(self) -> List[str]:
        return super().get_used_pins() + [self.sda, self.scl]

    def name(self) -> str:
        return f"I2c{self.raw_name}"


class SPI(CommunicationGroup):
    sck: str
    cipo: str
    copi: str

    def __init__(self, xml, comment):
        super().__init__(xml, comment)
        self.sck = xml.get("sck")
        self.cipo = xml.get("cipo")
        self.copi = xml.get("copi")

    def get_used_pins(self) -> List[str]:
        return super().get_used_pins() + [self.sck, self.cipo, self.copi]

    def name(self) -> str:
        return f"Spi{self.raw_name}"

Pll = namedtuple("Pll", ["m", "n", "p"])

class Clock:
    frequency: int
    crystal: int

    prescalars: Dict[str, int]
    pll: Pll

    def __init__(self, xml):
        self.frequency = int(xml.get("frequency"))
        self.crystal = int(xml.get("crystal"))

        self.prescalars = {clock.capitalize(): int(presc) for clock, presc in xml.find("prescalars").items() }
        self.pll = Pll(**{k: int(v) for k, v in xml.find("pll").items() })

class Controller:
    chip: str
    clock: Clock

    def __init__(self, xml):
        self.chip = xml.get("chip")
        self.clock = Clock(xml.find("clock"))

T = TypeVar("T")
class DeviceList(Generic[T]):
    aliases: Dict[str, T]
    names: Dict[Union[int, str], T]
    values: List[T]

    def __init__(self):
        self.values = []
        self.aliases = {}
        self.names = {}

    def named(self, name):
        if name not in self.names:
            return None
        return self.names[name]

    def aliased(self, alias):
        if alias not in self.aliases:
            return None
        return self.aliases[alias]

    def append(self, v):
        self.values.append(v)

        if v.alias is not None:
            assert v.alias not in self.aliases
            self.aliases[v.alias] = v
        
        assert v.raw_name not in self.names
        self.names[v.raw_name] = v

    def recompute_aliases(self):
        self.aliases = {
            v.alias: v for v in self.values
        }

    def __iter__(self):
        yield from sorted(self.values)

class BoardInfo:
    controller: Controller
    can: DeviceList[CanBus]
    uart: DeviceList[Uart]
    i2c: DeviceList[I2C]
    spi: DeviceList[SPI]
    gpio_groups: DeviceList[GpioGroup]
    gpio_pins: DeviceList[Gpio]
    adcs: List[Adc]
    timers: List[Timer]

    pin_to_usage: Dict[str, List[Instance]]

    def __init__(self, xml):
        self.controller = Controller(xml.find("controller"))
        self.can = DeviceList()
        self.uart = DeviceList()
        self.i2c = DeviceList()
        self.spi = DeviceList()
        self.gpio_groups = DeviceList()
        self.gpio_pins = DeviceList()

        self.updated = False

        comment = None

        for child in xml.iterchildren():
            if child.tag == "can":
                self.can.append(CanBus(child, comment))
            elif child.tag == "uart" or child.tag == "usart":
                self.uart.append(Uart(child, comment))
            elif child.tag == "i2c":
                self.i2c.append(I2C(child, comment))
            elif child.tag == "spi":
                self.spi.append(SPI(child, comment))
            elif child.tag == "gpio-group":
                self.gpio_groups.append(GpioGroup(child, comment))
            elif child.tag in ["gpio", "out", "in", "pwm", "analog"]:
                self.gpio_pins.append(Gpio(child, comment))
            elif isinstance(child, lxml.etree._Comment):
                comment = child.text
                continue
            elif child.tag != "controller":
                assert False, f"Unknown tag: {child.tag}"

            comment = None

        self.adcs = list(sorted(adcs.values()))
        self.timers = list(sorted(timers.values()))

        self.pin_to_usage = defaultdict(set)
        for instance in self.can.values + self.uart.values + self.i2c.values + self.spi.values + self.gpio_groups.values + self.gpio_pins.values:
            for pin in instance.get_used_pins():
                self.pin_to_usage[pin].add(instance)

    def validate_configuration(self, env):
        def extract_pin_defines(pins: str) -> str:
            pins = [pin.strip() for pin in str.split(pins, ",")]
            return [] if pins == [""] else pins

        digital_in_pins = extract_pin_defines(env[":board:digital_in_pins"])
        digital_out_pins = extract_pin_defines(env[":board:digital_out_pins"])
        analog_in_pins = extract_pin_defines(env[":board:analog_in_pins"])
        pwm_pins = extract_pin_defines(env[":board:pwm_pins"])

        pins = digital_in_pins + digital_out_pins + analog_in_pins + pwm_pins
        if len(pins) != len(set(pins)):
            raise ValidateException("Duplicate pin definitions")

        for pin in pins:
            usages = self.pin_to_usage[self.gpio_pins.aliased(pin).raw_name]

            for usage in usages:
                if type(usage) != Gpio:
                    if usage.alias != None:
                        raise ValidateException(f"Pin {pin} has multiple enforced usages: {usages}")

    def update_env(self, env):
        if self.updated:
            return

        for instance in self.i2c.values + self.spi.values + self.uart.values:
            if instance.alias is None:
                alias = env[f":board:{instance.display_name().lower()}_alias"]

                if alias != "":
                    instance.alias = alias
                else:
                    instance.alias = instance.display_name()

        self.i2c.recompute_aliases()
        self.uart.recompute_aliases()
        self.spi.recompute_aliases()

        self.updated = True

    def print_gpio_af_information(self):
        device = get_modm_device(self.controller.chip)
        device_gpios_raw = device.properties["driver"][-1]["gpio"]
        device_gpios = {}
        for p in device_gpios_raw:
            gpio = "Gpio" + p["port"].capitalize() + p["pin"]
            if "signal" in p.keys():
                device_gpios[gpio] = p["signal"]
            else:
                device_gpios[gpio] = []

        for gpio in self.gpio_pins:
            afs = device_gpios[gpio.raw_name]
            print(gpio.alias, ": ", gpio.raw_name)

            for af in afs:
                if af["driver"] == "tim":
                    if "ch" in af["name"]:
                        print("\t", "Timer" + af["instance"], af["name"])
                if af["driver"] == "adc":
                    print("\t", "Adc" + af["instance"], af["name"])

    @lru_cache
    def get_all_gpio_pins(self):
        pins = {}
        for pin in self.gpio_pins:
            pins[pin.display_name()] = pin

        for group in self.gpio_groups.values + self.spi.values + self.i2c.values:
            for pin in group.gpios:
                pins[pin.display_name()] = pin

        return pins

    @lru_cache
    def get_enabled_peripherals(self):
        enabled_peripherals = defaultdict(set)
        for can in self.can:
            enabled_peripherals["Can"].add(can)
        for i2c in self.i2c:
            enabled_peripherals["I2c"].add(i2c)
        for spi in self.spi:
            enabled_peripherals["Spi"].add(spi)
        for uart in self.uart:
            enabled_peripherals["Uart"].add(uart)
        for adc in self.adcs:
            enabled_peripherals["Adc"].add(adc)
        for timer in self.timers:
            enabled_peripherals["Timer"].add(timer)

        for key, value in enabled_peripherals.items():
            enabled_peripherals[key] = sorted(list(value))

        return enabled_peripherals


@lru_cache
def get_modm_device(chip):
    import importlib
    modm_devices = importlib.machinery.SourceFileLoader("modm_devices", str(repo_path_rel_repolb(__file__, "./modm/ext/modm-devices/modm_devices/__init__.py"))).load_module()

    device = None
    for filename in glob.glob(str(repo_path_rel_repolb(__file__, f"./modm/ext/modm-devices/devices/stm32/{chip[:7]}-*.xml"))):
        for d in modm_devices.parser.DeviceParser().parse(filename).get_devices():
            if d.partname == chip:
                device = d
                break

    return device


def parse_board_info(device, env=None):
    global parsed_board_info

    device_file_names = glob.glob(str(repo_path_rel_repolb(__file__, "supported-devices/*.xml")))
    device_file_names = [dfn for dfn in device_file_names if device in dfn]
    assert len(device_file_names) == 1, f"Device {device} not found or there are multiple device files with the device name"
    device = device_file_names[0]

    if device not in parsed_board_info:
        # parse the xml-file if we haven't already
        parser = lxml.etree.XMLParser(no_network=True)
        xmlroot = lxml.etree.parse(device_file_names[0], parser=parser)
        xmlroot.xinclude()
        parsed_board_info[device] = BoardInfo(xmlroot.getroot())

    if env is not None:
        parsed_board_info[device].update_env(env)

    return parsed_board_info[device]
