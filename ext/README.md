## Integrating Third Party Libraries

This directory contains all third party libraries that taproot depends on. If you would like to add
a new third-party library, you should add it as a submodule in this directory. In particular, set
up the library as follows:

1. Add a new directory in the `ext/` directory. If the third party library is called `<cool-lib>`,
   call the directory `<cool-lib>-project`.
1. Create a `module.lb` file in `<cool-lib>-project`. This will be used to copy files from the
   library upon code generation.
1. The file should look like the following. Replace `<cool-lib>` with the third party library and
   `<cool-file>` with the third party library file(s) to copy when generating software:

   ```
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

    def init(module):
        module.name = ":ext:<cool-lib>"
        module.description = "<A good description>"

    def prepare(module, options):
        return True

    def build(env):
        # Copy all folders and files
        env.outbasepath = "taproot/ext"
        # Copy files from the submodule to the ext directory
        env.copy("<cool-lib>/<cool-file>")
   ```
