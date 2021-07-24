#!/usr/bin/env python3
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

import os
import sys

SOURCE_FILE_EXTENSIONS = ['.cpp', '.hpp', '.h']

if len(sys.argv) != 2:
    print("usage: /usr/bin/python3 check_license_headers.py <project-src-path>")
    sys.exit(2)

project_path = sys.argv[1]

string_to_check = 'DoNotUse_getDrivers'

files_to_whitelist = [
    'aruwlib/drivers_singleton.cpp',
    'aruwlib/drivers_singleton.hpp',
    'main.cpp'
]

# Add on the project_path to all files in files_to_whitelist
files_to_whitelist = [ os.path.join(project_path, f) for f in files_to_whitelist ]
# Find all files in project_path
files_to_search = [ os.path.join(dp, f) for dp, dn, filenames in os.walk(project_path) for f in filenames ]


def is_source_file_to_check(file, ignored_files):
    if file in ignored_files:
        return False

    _, file_extension = os.path.splitext(file)
    return file_extension in SOURCE_FILE_EXTENSIONS

def check_file(file):
    '''
    Checks if 'file' contains 'string_to_check' and returns true if it does.
    '''
    with open(file, 'r') as file_to_check:
        if string_to_check in file_to_check.read():
            print("{0} contains the function call {1}".format(file, string_to_check))
            return True
    return False

# Iterate through all files, checking for the 'string_to_check'
result = 0
for file in files_to_search:
    if is_source_file_to_check(file, files_to_whitelist):
        result = check_file(file) or result

sys.exit(result)
