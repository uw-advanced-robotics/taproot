[![pipeline status](https://gitlab.com/aruw/controls/aruwlib/badges/develop/pipeline.svg)](https://gitlab.com/aruw/controls/aruwlib/-/commits/develop)
[![coverage report](https://gitlab.com/aruw/controls/aruwlib/badges/develop/coverage.svg)](https://gitlab.com/aruw/controls/aruwlib/-/commits/develop)
[![discord invite](https://discord.com/api/guilds/856735962663223306/widget.png)](https://discord.gg/jjDrGhrjMy)
[![Open in Visual Studio Code](https://open.vscode.dev/badges/open-in-vscode.svg)](vscode://ms-vscode-remote.remote-containers/cloneInVolume?url=https://gitlab.com/aruw/controls/aruwlib)

# aruwlib

A friendly control library and framework for [RoboMaster](https://www.robomaster.com/en-US) robots.

aruwlib provides a comprehensive, convenient and well-tested set of APIs designed specifically for
the RoboMaster robotics competition. Key functionality and features include:
- Drivers for common RoboMaster/DJI hardware:
   - RoboMaster motors connected via CAN: C620, GM3508, GM3510 and others
   - DR16 Receiver
   - RoboMaster referee system via UART
- An organizational framework and round-robin scheduler taking inspiration from FIRST's [wpilib](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- Common building blocks such as:
   - Setpoint-based subsystems with jam detection logic and calibration
   - Closed-loop power limiting
   - Filtering, smoothing and ramping primitives
   - Computational helpers for wrapping measurements such as angles
- Built-in error aggregation and reporting via optional UART terminal or OLED display
- Monitoring of key subsystems and status information via optional UART terminal or OLED display
- First-class support for unit and integration testing (no hardware required)

aruwlib is a C++ library targeting modern C++ (C++17).

We use:
- `modm`, a C++-native HAL
- the gcc compiler
- openocd to deploy and debug
- VSCode, an editor

This library was originally designed for use in Advanced Robotics at the University of Washington
("ARUW")'s robot controls code, which remains the largest consumer. You can find the project
[here](https://gitlab.com/aruw/controls/aruw-mcb).

## Getting started

TODO. Check out the [template project](https://gitlab.com/aruw/controls/aruwlib-template-project).

## Resources
- The [generated API documentation for aruwlib](https://aruw.gitlab.io/controls/aruwlib/)
- [aruw-mcb](https://gitlab.com/aruw/controls/aruw-mcb), ARUW's full robot code project
- [aruw-edu](https://gitlab.com/aruw/controls/aruw-edu): a hands-on tutorial for building robot code with aruwlib
- The [template project](https://gitlab.com/aruw/controls/aruwlib-template-project) to kick-start your own development
- The [aruwlib wiki](https://gitlab.com/aruw/controls/aruwlib/-/wikis/home)
- The [modm website](https://modm.io/) and associated documentation

## Requirements for use

TODO: move to aruwlib-template-project?

- A robot operated by a [RoboMaster Development Board Type A](https://store.dji.com/product/rm-development-board-type-a).
   - Support for the RoboMaster Development Board Type C is planned, tracked here: https://gitlab.com/aruw/controls/aruwlib/-/issues/9
- A Linux environment. Virtual Machines work great. Documentation forthcoming (TODO).
- An ST-Link- or J-Link-compatible probe for flashing and debugging

## Contacting

If you have any questions please file an Issue or join our Discord server (linked above). We can
also be reached privately at robomstr@uw.edu.

## Licensing

aruwlib is covered under the GPL-3.0-or-later with the following exceptions:
- The `/modm` submodule is licensed under MPL 2.0 by the modm project. We _are not_
  the license holder for these files. See `/modm/LICENSE` for license information.
- `src/aruwlib/algorithms/MahonyAHRS.h` and `src/aruwlib/algorithms/MahonyAHRS.cpp` are licensed
  under the GPL by SOH Madgwick. The repo containing this code can be found
  [here](https://github.com/uw-advanced-robotics/MahonyAHRS).

Other RoboMaster teams are invited, and encouraged, to utilize this library. We have licensed
aruwlib and the template project under the GPL to encourage collaboration and open publishing of
RoboMaster controls codebases. We politely request that other teams choosing to utilize this
library, or parts of it (including its design), open-source their own code in turn.

## Development guide

TODO: we are looking to select and fully document a recommended workflow. See https://gitlab.com/aruw/controls/aruwlib/-/issues/15.

We provide a Visual Studio Code development container environment to ease development in this repo.
This is the easiest way to work on aruwlib. See [here](https://code.visualstudio.com/docs/remote/containers)
for setup instructions. Once you have set up the tool, you can use the vscode command
"Remote-Containers: Clone Repository in Container Volume" and provide it the URL
`https://gitlab.com/aruw/controls/aruwlib.git` to open this project in a container.

You will then need to initialize submodules to pull down modm:

```
git submodule update --init --recursive
```

If you would instead prefer to configure your own environment, we recommend you refer to the
[Dockerfile for our development container](https://gitlab.com/aruw/controls/aruw-mcb-dev-container/-/blob/master/Dockerfile)
for the necessary setup. 

### Contributing

We welcome Merge Requests and Issues! Feel free to take on an open Issue if you see one valuable to
you. We recommend posting to let us know what you're working on so we don't duplicate effort.

Issues and our Discord server are both great ways to get in touch.

When you create a new branch, always branch off of `develop`.

### Branch naming conventions in the aruwlib repository

Names should follow the format `FirstL/#{Issue Number}/short-description`. For example:
`RyanT/#0/linter-integration`.

### Building and testing via the terminal

This library is configurable via `lbuild` parameters, and consumers use this tool to generate a full
copy of the files in aruwlib in their own projects. As such, there is no "one true configuration"
for consumers using aruwlib.

To facilitate testing, this repo has a project defined in `test-project/` which consumes aruwlib. It
can be used for basic testing and as a generation/build smoke-test. However, changes made by ARUW
members should typically be tested in the context of `aruw-mcb` before being merged here.

To use the test project, `cd` into `aruwlib/test-project` (where the `project.xml` file is). You
will have to first use `lbuild` to generate an aruwlib distribution before trying to build.

- `lbuild build`: Re-generates our copy of modm according to the modules specified in `project.xml`. Note that there is a _separate_ instance used for the unit tests, which can be build by runnint the same command from within the `sim-modm` subdirectory.
- `scons build`: Builds the firmware image for the hardware target. Creates a "release" folder located in `build/hardware/` which contains the final `.elf` file as well as the intermediate object files (`.o`).
- `scons build-tests`: Builds a program which hosts our unit tests. This executable can be run on your host computer (only supported on Linux) and prints results for each unit test run.
- `scons program`: Builds as with `scons build` and then programs the board.
- `scons run-tests`: Builds and runs the unit test program.
- `scons size`: Prints statistics on program size and (satically-)allocated memory. Note that the reported available heap space is an upper bound, and this tool has no way of knowing about the real size of dynamic allocations.

## Working with modm

### What is modm?

We use an embedded library generator called modm in our codebase. It will eventually be important
that you understand what how modm works. For now you can just think about it as handling lower level
IO on our MCB. You should read [modm's homepage](https://modm.io/) so you have a general idea of
what it does.

### Modm examples

The modm website provides a great number of examples that can be very useful when interacting with
modm's hardware architecture layer for the first time. The examples are located on modm's website
[here](https://modm.io/#examples).

### Adding new dependencies on modm modules (advanced)

Look up the fully-qualified name of the module from the [modm website](https://modm.io/reference/module/modm-architecture/)
or select one from the output of `lbuild discover`. The name will look like `:platform:gpio`. Open
`aruwlib/build_tools/project.xml.in` and add an entry to the dependencies section like the
following:

```xml
<module>modm:platform:gpio</module>
```

Now open the terminal, `cd` into `test-project`, and run `lbuild build`.
