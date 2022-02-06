#!/usr/bin/python3
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

from os.path import join, dirname, splitext
import sys
import glob


USAGE = "usage: /usr/bin/python3 check_license_headers.py [--update] \n\
options:\n\
    --update Adds licenses to files that don't have a license header (optional)"


FILE_GLOBS_TO_IGNORE = [
    '../modm/**/*',
    '../**/test-project/taproot/**/*',
    '../docs/*',
    '../**/__init__.py',
    '../src/tap/algorithms/MahonyAHRS.*']
SCRIPT_DIR = dirname(__file__)

CPP_LICENSED_SOURCE_FILE_EXTENSIONS = ['.cpp', '.hpp', '.h', '.hpp.in', '.cpp.in']
CPP_LICENSE_HEADER = '/*\n\
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>\n\
 *\n\
 * This file is part of Taproot.\n\
 *\n\
 * Taproot is free software: you can redistribute it and/or modify\n\
 * it under the terms of the GNU General Public License as published by\n\
 * the Free Software Foundation, either version 3 of the License, or\n\
 * (at your option) any later version.\n\
 *\n\
 * Taproot is distributed in the hope that it will be useful,\n\
 * but WITHOUT ANY WARRANTY; without even the implied warranty of\n\
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n\
 * GNU General Public License for more details.\n\
 *\n\
 * You should have received a copy of the GNU General Public License\n\
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.\n\
 */\n'

SCRIPT_LICENSED_SOURCE_FILE_EXTENSIONS = ['.lb', '.py', '.sh', '.yml', '.py.in']
SCRIPT_LICENSE_HEADER = '# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>\n\
#\n\
# This file is part of Taproot.\n\
#\n\
# Taproot is free software: you can redistribute it and/or modify\n\
# it under the terms of the GNU General Public License as published by\n\
# the Free Software Foundation, either version 3 of the License, or\n\
# (at your option) any later version.\n\
#\n\
# Taproot is distributed in the hope that it will be useful,\n\
# but WITHOUT ANY WARRANTY; without even the implied warranty of\n\
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n\
# GNU General Public License for more details.\n\
#\n\
# You should have received a copy of the GNU General Public License\n\
# along with Taproot.  If not, see <https://www.gnu.org/licenses/>.\n'
COPYRIGHT = "Copyright (c)"
AUTHOR = "Advanced Robotics at the University of Washington <robomstr@uw.edu>"
SHEBANG = "!"

def find_files_to_check():
    file_types_to_check = CPP_LICENSED_SOURCE_FILE_EXTENSIONS + SCRIPT_LICENSED_SOURCE_FILE_EXTENSIONS

    files_to_check = []
    for file_type in file_types_to_check:
        files_to_check.extend(glob.glob(join(SCRIPT_DIR, '../**/*{}'.format(file_type)), recursive=True))

    files_to_ignore = []
    for file_glob in FILE_GLOBS_TO_IGNORE:
        files_to_ignore.extend(glob.glob(join(SCRIPT_DIR, file_glob), recursive=True))

    if files_to_ignore:
        def excluded(file):
            return file in files_to_ignore

    return list(filter(lambda p: not excluded(p), files_to_check))

def parse_args():
    update_files = sys.argv[-1] == "--update"
    return update_files

def is_licensed_source_file(file, file_extensions):
    _, file_extension = splitext(file)
    return file_extension in file_extensions

def file_has_valid_license_header(file, expected_header, copyright_line):
    with open(file, 'r') as file_to_check:
        license_lines = expected_header.splitlines()
        num_lines = len(license_lines)
        license_lines_to_check = file_to_check.read().splitlines()
        # shebang lines may appear at the top of the file; skip over these
        offset = 0
        while SHEBANG in license_lines_to_check[offset]:
            offset += 1
        # accomodate for one blank line after shebang lines, if any
        if offset != 0:
            offset += 1
        # check before copyright line
        if license_lines_to_check[offset:offset + copyright_line] != license_lines[0:copyright_line]:
            return False
        # check after copyright line
        if license_lines_to_check[offset + copyright_line + 1:offset + num_lines] != license_lines[copyright_line + 1:]:
            return False
        # check copyright line
        if COPYRIGHT not in license_lines_to_check[offset + copyright_line] or AUTHOR not in license_lines_to_check[offset + copyright_line]:
            return False
    return True

def add_license_to_file(file, header):
    print("Adding license to {0}".format(file))
    with open(file, 'r+') as file_to_check:
        content = file_to_check.read()
        file_to_check.seek(0, 0)
        file_to_check.write(header.rstrip('\r\n') + '\n' + content)

def main():
    if len(sys.argv) not in [ 1, 2 ]:
        print(USAGE)
        sys.exit(2)
    
    update_files = parse_args()

    files_to_check = find_files_to_check()

    result = False
    for file in files_to_check:
        if is_licensed_source_file(file, CPP_LICENSED_SOURCE_FILE_EXTENSIONS):
            if not file_has_valid_license_header(file, CPP_LICENSE_HEADER, 1):
                result = True
                print("{0} does not contain a license header".format(file))
                if update_files:
                    add_license_to_file(file, CPP_LICENSE_HEADER)
        elif is_licensed_source_file(file, SCRIPT_LICENSED_SOURCE_FILE_EXTENSIONS):
            if not file_has_valid_license_header(file, SCRIPT_LICENSE_HEADER, 0):
                result = True
                print("{0} does not contain a license header".format(file))
                if update_files:
                    add_license_to_file(file, SCRIPT_LICENSE_HEADER)

    sys.exit(result)

if __name__ == '__main__':
    main()
