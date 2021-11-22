#!/bin/bash
#
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
#
# This script builds the hardware profile for each robot target.
# Assumes you are in the build directory (where the SConstruct file is) 

set -e

targets="\
    TARGET_SOLDIER \
    TARGET_OLD_SOLDIER \
    TARGET_ENGINEER \
    TARGET_HERO \
    TARGET_SENTINEL \
    TARGET_DRONE"

for target in $targets; do
    echo ===============================================================================
    echo building $target
    echo ===============================================================================
    /usr/bin/env python3 $(which scons) build robot=$target additional-ccflags=-Werror
done
