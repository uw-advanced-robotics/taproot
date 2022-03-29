#!/usr/bin/python3
#
# Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

import argparse
import os
import re
from sre_constants import SUCCESS

import utils

HEADER_EXTENSIONS = [ '.hpp', '.h' ]

def parse_args():
    arg = argparse.ArgumentParser(
        description='Script that validates header guards have valid structure. A valid header\
            guard has the form <PREFIX>FILE_NAME_[H|HPP]_, where <PREFIX> is an optional string\
            and FILE_NAME is the name of the file with all upper cases. Checks that all header\
            guards are present in all files and that there is nothing meaningful before/after\
            the header guard (aside from comments/whitespace). If there are particular headers\
            that don\'t need a header guard, specify as such with the optional parameter -i.')
    arg.add_argument(
        'directories',
        nargs='+',
        help='directories that should be searched when validating header guards')
    arg.add_argument(
        '-p', '--prefix',
        help='regex string that all header guards should match',
        required=False)
    arg.add_argument(
        '-i', '--ignored',
        nargs='*',
        help='files to ignore when validating header guards',
        required=False)
    return arg.parse_args()

def get_header_guard(prefix: str, filename: str):
    return prefix + filename.replace('.', '_').upper() + '_'

def main():
    args = parse_args()

    files_in_dir: list = []
    for dir in args.directories:
        files_in_dir.extend([ os.path.normpath(os.path.join(dp, f)) for dp, _, filenames in os.walk(dir) for f in filenames ])

    files_to_ignore = [ os.path.normpath(fn) for fn in args.ignored ]

    files_to_validate: list = []
    for file in files_in_dir:
        if file in files_to_ignore:
            continue

        split = os.path.splitext(file)
        if len(split) > 1 and split[1] in HEADER_EXTENSIONS:
            files_to_validate.append(file)

    exit_code = 0

    for file in files_to_validate:
        with open(file, 'r') as f:
            header_guard: str = get_header_guard(args.prefix, os.path.basename(file))
            lines: list = utils.emptyline_remover(utils.comment_remover(f.read()))
            if lines[0].strip('\n') != '#ifndef ' + header_guard or lines[1].strip('\n') != '#define ' + header_guard:
                print(f'error: failed to find valid header guard in {file}, expected header guard {header_guard}')
                exit_code = 1

    exit(exit_code)

if __name__ == '__main__':
    main()
