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

parsed_board_info = {}

class Instance(ABC):
    name: Union[str, int]
    alias: Optional[str]
    comment: Optional[str]

    def __init__(self, xml, comment):
        name = xml.get("name")
        self.name = int(name) if name.isnumeric() else name
        
        self.alias = xml.get("alias")
        self.comment = comment.strip() if comment is not None else None

    def display_name(self):
        return self.alias if self.alias is not None else str(self.name)

    def __repr__(self):
        return self.display_name()

    @abstractmethod
    def get_used_pins(self) -> List[str]:
        ...


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

class Feature(ABC):
    @staticmethod
    def parse(xml):
        if xml.tag == "adc":
            return AdcFeature.parse(xml)
        elif xml.tag == "timer":
            return TimerFeature.parse(xml)

class AdcFeature(Feature):
    name: int
    in_channel: str

    @staticmethod
    def parse(xml):
        assert xml.tag == "adc"

        name = int(xml.get("name")[3:])
        in_channel = xml.get("in")

        return AdcFeature(name, in_channel)

    def __init__(self, name, in_channel):
        self.name = name
        self.in_channel = in_channel

class TimerFeature(Feature):
    name: int
    channel: str

    @staticmethod
    def parse(xml):
        assert xml.tag == "timer"

        name = int(xml.get("name")[5:])
        channel = xml.get("channel")

        return TimerFeature(name, channel)

    def __init__(self, name, channel):
        self.name = name
        self.channel = channel

class Gpio(Instance):
    features: Dict[str, Feature]
    gpio_type: str

    def __init__(self, xml, comment):
        assert xml.tag in  ["gpio", "out", "in", "pwm", "analog"]
        super().__init__(xml, comment)

        self.features = {child.tag: Feature.parse(child) for child in xml.iterchildren()}

        self.gpio_type = xml.tag

        if self.gpio_type == "analog":
            self.features["adc"] = AdcFeature(int(xml.get("adc")[3:]), xml.get("in"))
        elif self.gpio_type == "pwm":
            self.features["timer"] = TimerFeature(int(xml.get("timer")[5:]), xml.get("channel"))

    def get_used_pins(self) -> List[str]:
        return [self.name]


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
        return [pin.name for pin in self.gpios]


class CommunicationGroup(Instance):
    gpios: List[Gpio]

    def __init__(self, xml, comment):
        super().__init__(xml, comment)
        self.name = int(xml.get("name"))
        self.alias = xml.get("alias", None)

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

    def aliases(self, alias):
        if alias not in self.aliases:
            return None
        return self.aliases[alias]

    def append(self, v):
        self.values.append(v)

        if v.alias is not None:
            assert v.alias not in self.aliases
            self.aliases[v.alias] = v
        
        assert v.name not in self.names
        self.names[v.name] = v

    def __iter__(self):
        yield from self.values

class BoardInfo:
    controller: Controller
    can: DeviceList[CanBus]
    uart: DeviceList[Uart]
    i2c: DeviceList[I2C]
    spi: DeviceList[SPI]
    gpio_groups: DeviceList[GpioGroup]
    gpio_pins: DeviceList[Gpio]

    def __init__(self, xml):
        self.controller = Controller(xml.find("controller"))
        self.can = DeviceList()
        self.uart = DeviceList()
        self.i2c = DeviceList()
        self.spi = DeviceList()
        self.gpio_groups = DeviceList()
        self.gpio_pins = DeviceList()

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

        self.pin_to_usage = defaultdict(set)
        for instance in self.can.values + self.uart.values + self.i2c.values + self.spi.values + self.gpio_groups.values + self.gpio_pins.values:
            for pin in instance.get_used_pins():
                self.pin_to_usage[pin].add(instance)

        self.populate_gpio_information()

    def populate_gpio_information(self):
        device = get_modm_device(self.controller.chip)
        device_gpios_raw = device.properties["driver"][-1]["gpio"]
        device_gpios = {}
        for p in device_gpios_raw:
            # print(p)
            gpio = "Gpio" + p["port"].capitalize() + p["pin"]
            # print(p["signal"])
            if "signal" in p.keys():
                device_gpios[gpio] = p["signal"]
            else:
                device_gpios[gpio] = []

        for gpio in self.gpio_pins:
            afs = device_gpios[gpio.name]
            print(gpio.alias, ": ", gpio.name)

            for af in afs:
                if af["driver"] == "tim":
                    if "ch" in af["name"]:
                        print("\t", "Timer" + af["instance"], af["name"])
                        # gpio.features["timer"].append(TimerFeature(int(af["instance"]), af["name"]))
                if af["driver"] == "adc":
                    print("\t", "Adc" + af["instance"], af["name"])
                    # if "adc" not in gpio.features:
                    #     gpio.features["adc"].append(AdcFeature({int(af["instance"])}, af["name"].capitalize()))
                    # elif af["name"].capitalize() == gpio.features["adc"][0].in_channel:
                    #     gpio.features["adc"][0].name.add(int(af["instance"]))
                    # else:
                    #     print("Found two ADCs with different channels, currently not supported")
                    #     print(afs)

    @lru_cache
    def get_all_gpio_pins(self):
        pins = {}
        for pin in self.gpio_pins:
            name = pin.alias if pin.alias is not None else pin.name
            pins[name] = pin

        for group in self.gpio_groups.values + self.spi.values + self.i2c.values:
            for pin in group.gpios:
                # name = pin.alias if pin.alias is not None else pin.name
                pins[pin.display_name()] = pin
        
        return pins


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


def parse_board_info(device):
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

    return parsed_board_info[device]
