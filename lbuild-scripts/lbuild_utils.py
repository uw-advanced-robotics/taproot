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

import os

from pathlib import Path

def repo_path_rel_repolb(file, path):
    """
    Relocate given path to the path of the repo file.
    Copied from `modm/test/all/run_all.py`

    - file: __file__ that the function is called in
    - path: path relative to repo.lb file that you want get
    """
    return (Path(os.path.abspath(file)).parents[1] / path)
