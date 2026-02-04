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
from typing import List, Optional, Dict
from enum import Enum
from abc import ABC
from collections import namedtuple

parsed_board_info = {}

class CanBus:
    name: int
    rx: str
    tx: str

    def __init__(self, xml):
        assert xml.tag == "can"
        
        self.name = int(xml.get("name"))
        self.rx = xml.get("rx")
        self.tx = xml.get("tx")


class Uart:
    name: int
    usart: bool
    rx: Optional[str]
    tx: Optional[str]

    def __init__(self, xml):
        assert xml.tag == "uart" or xml.tag == "usart"

        self.name = int(xml.get("name"))
        self.usart = xml.tag == "usart"
        self.rx = xml.get("rx", None)
        self.tx = xml.get("tx", None)


GpioType = Enum("GpioType", ["OUTPUT", "INPUT", "CUSTOM"])


class Feature(ABC):
    @staticmethod
    def parse(xml):
        if xml.tag == "adc":
            return AdcFeature(xml)
        elif xml.tag == "timer":
            return TimerFeature(xml)


class AdcFeature(Feature):
    name: int
    in_channel: str

    def __init__(self, xml):
        assert xml.tag == "adc"

        self.name = int(xml.get("name")[3:])
        self.in_channel = xml.get("in")


class TimerFeature(Feature):
    name: int
    channel: str

    def __init__(self, xml):
        assert xml.tag == "timer"

        self.name = int(xml.get("name")[5:])
        self.channel = xml.get("channel")

class Gpio:
    alias: Optional[str]
    name: str
    features: Dict[str, Feature]
    typ: GpioType

    def __init__(self, xml):
        assert xml.tag == "gpio" or xml.tag == "out" or xml.tag == "in"

        self.name = xml.get("name")
        self.alias = xml.get("alias", None)
        self.features = {child.tag: Feature.parse(child) for child in xml.iterchildren()}

        self.typ = GpioType.OUTPUT if xml.tag == "out" else \
                GpioType.INPUT if xml.tag == "in" else \
                GpioType.CUSTOM


class GpioGroup:
    alias: str
    default: Optional[str]
    comment: Optional[str]
    gpios: List[Gpio]

    def __init__(self, xml):
        assert xml.tag == "gpio-group"

        self.alias = xml.get("alias")
        self.default = xml.get("default", None)
        self.comment = xml.get("comment", None)

        self.gpios = [Gpio(child) for child in xml.iterchildren()]


class CommunicationGroup:
    name: int
    alias: Optional[str]
    gpios: List[Gpio]

    def __init__(self, xml):
        self.name = int(xml.get("name"))
        self.alias = xml.get("alias", None)

        self.gpios = [Gpio(child) for child in xml.iterchildren()]


class I2C(CommunicationGroup):
    sda: str
    scl: str

    def __init__(self, xml):
        super().__init__(xml)
        self.sda = xml.get("sda")
        self.scl = xml.get("scl")


class SPI(CommunicationGroup):
    sck: str
    cipo: str
    copi: str

    def __init__(self, xml):
        super().__init__(xml)
        self.sck = xml.get("sck")
        self.cipo = xml.get("cipo")
        self.copi = xml.get("copi")

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

class BoardInfo:
    controller: Controller
    can: List[CanBus]
    uart: List[Uart]
    i2c: List[I2C]
    spi: List[SPI]
    gpio_groups: List[GpioGroup]
    gpio_pins: List[Gpio]

    def __init__(self, xml):
        self.controller = Controller(xml.find("controller"))
        self.can = []
        self.uart = []
        self.i2c = []
        self.spi = []
        self.gpio_groups = []
        self.gpio_pins = []

        for child in xml.iterchildren():
            if child.tag == "can":
                self.can.append(CanBus(child))
            elif child.tag == "uart" or child.tag == "usart":
                self.uart.append(Uart(child))
            elif child.tag == "i2c":
                self.i2c.append(I2C(child))
            elif child.tag == "spi":
                self.spi.append(SPI(child))
            elif child.tag == "gpio-group":
                self.gpio_groups.append(GpioGroup(child))
            elif child.tag == "gpio":
                self.gpio_pins.append(Gpio(child))
            elif child.tag != "controller":
                assert False, f"Unknown tag: {child.tag}"


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
