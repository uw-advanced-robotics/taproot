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
import os

from pathlib import Path

def repopath(path):
    return (Path(os.path.abspath(__file__)).parents[2] / path)

class BoardInfoParser:
    parsed_board_info = {}

    @staticmethod
    def parse_board_info(device):
        device_file_names = glob.glob(str(repopath("supported-devices/*.xml")))
        device_file_names = [dfn for dfn in device_file_names if device in dfn]
        assert len(device_file_names) == 1, f"Device {device} not found or there are multiple device files with the device name"
        device = device_file_names[0]

        try:
            BoardInfoParser.parsed_board_info[device]
        except KeyError:
            # parse the xml-file
            parser = lxml.etree.XMLParser(no_network=True)
            xmlroot = lxml.etree.parse(device_file_names[0], parser=parser)
            xmlroot.xinclude()
            BoardInfoParser.parsed_board_info[device] = xmlroot.getroot()

        return BoardInfoParser.parsed_board_info[device]
