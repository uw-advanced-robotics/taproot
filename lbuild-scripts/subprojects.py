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

def build_subproject(name, cwd, git_dir):
    print(f"building {name}")
    try:
        git_sha = subprocess.check_output(["git", "describe", "--always"], cwd=git_dir).decode('UTF-8').strip()
        directory_sha = hash_directory(cwd)

        if os.path.exists(os.path.join(cwd, ".cache")):
            with open(os.path.join(cwd, ".cache"), "rb") as f:
                data = f.read(65536).decode('UTF-8').strip()
                if data == f"{git_sha} {directory_sha}":
                    return

        subprocess.run(["lbuild", "build"], check=True, cwd=cwd)
        with open(os.path.join(cwd, ".cache"), "wb") as f:
            f.write(f"{git_sha} {directory_sha}".encode("UTF-8"))
    except subprocess.CalledProcessError as e:
        print(e)
        exit(1)

def hash_directory(directory):
    import multiprocessing 
    files_to_hash = []
    hashed_files = []
    for root, _, files in os.walk(directory, topdown=True):
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
            sha.update(data)
    return sha.hexdigest()