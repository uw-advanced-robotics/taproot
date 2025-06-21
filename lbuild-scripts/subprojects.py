# Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

import subprocess
import os
import hashlib
import re
import platform

def build_subproject(name, cwd, output=""):
    print(f"building {name}")

    if output == "":
        output = cwd

    try:
        git_dir = ""

        if platform.system() == "Windows":
            override_windows(cwd)

        with open(os.path.join(cwd, "project.xml"), "rb") as f:
            data = f.read(65536).decode('UTF-8').strip()
            git_dir = re.findall(r"<path>(.*)(\/|\\)repo\.lb</path>", data)[0][0]

        git_sha = subprocess.check_output(["git", "describe", "--always", "--abbrev=7"], cwd=os.path.join(cwd, git_dir)).decode('UTF-8').strip()
        directory_sha = hash_directory(cwd, output)

        if os.path.exists(os.path.join(cwd, ".cache")):
            with open(os.path.join(cwd, ".cache"), "rb") as f:
                data = f.read(65536).decode('UTF-8').strip()
                if data == f"{git_sha} {directory_sha}":
                    return

        subprocess.run(["lbuild", "build"], check=True, cwd=cwd)

        if platform.system() == "Windows":
            override_windows(cwd)

        directory_sha = hash_directory(cwd, output)
        with open(os.path.join(cwd, ".cache"), "wb") as f:
            f.write(f"{git_sha} {directory_sha}".encode("UTF-8"))
    except Exception as e:
        print(e)
        exit(1)

def hash_directory(cwd, output):
    import multiprocessing 
    files_to_hash = []
    hashed_files = []

    if output != cwd:
        files_to_hash.append(os.path.join(cwd, "project.xml"))

    for root, _, files in os.walk(output, topdown=True):
        files.sort()

        for file in files:
            if file.endswith(".log") or file.endswith(".cache"):
                continue
            files_to_hash.append(os.path.join(root, file))

    with multiprocessing.Pool() as pool:
        hashed_files += pool.map(hash_file, files_to_hash)

    sha = hashlib.sha1()
    for hash in sorted(hashed_files):
        sha.update(hash.encode("UTF-8"))
    return sha.hexdigest()

def hash_file(file):
    sha = hashlib.sha1()

    with open(file, 'rb') as f:
        while True:
            data = f.read(65536)
            if not data:
                break
            data = data.replace(b"\r\n", b"\n")
            sha.update(data)
    return sha.hexdigest()

def override_windows(cwd):
    # Note: The LF/CRLF change should be undone by git automatically when the change is staged but we do it manually to reduce confusion
    LF_TO_CRLF = ["modm/ext/gcc/cabi.c"]
    DOUBLE_BACKSLASHES_TO_FORWARD_SLASHES = ["modm/openocd.cfg"]
    BACKSLASHES_TO_FORWARD_SLASHES = [
        "project.xml",
        "modm/SConscript",
        "modm/ext/printf/printf.h",
    ]

    for file_path in LF_TO_CRLF:
        if os.path.exists(os.path.join(cwd, file_path)):
            with open(os.path.join(cwd, file_path), "rb+") as f:
                content = f.read()
                content = content.replace(b"\r\n", b"\n")
                f.seek(0)
                f.write(content)
                f.truncate()
    
    for file_path in DOUBLE_BACKSLASHES_TO_FORWARD_SLASHES:
        if os.path.exists(os.path.join(cwd, file_path)):
            with open(os.path.join(cwd, file_path), "r+", encoding="utf8") as f:
                content = f.read()
                content = content.replace("\\\\", "/")
                f.seek(0)
                f.write(content)
                f.truncate()

    for file_path in BACKSLASHES_TO_FORWARD_SLASHES:
        if os.path.exists(os.path.join(cwd, file_path)):
            with open(os.path.join(cwd, file_path), "r+", encoding="utf8") as f:
                content = f.read()
                content = content.replace("\\", "/")
                f.seek(0)
                f.write(content)
                f.truncate()

