[![pipeline status](https://gitlab.com/aruw/controls/taproot/badges/develop/pipeline.svg)](https://gitlab.com/aruw/controls/taproot/-/commits/develop)
[![coverage report](https://gitlab.com/aruw/controls/taproot/badges/develop/coverage.svg)](https://gitlab.com/aruw/controls/taproot/-/commits/develop)
[![discord invite](https://discord.com/api/guilds/856735962663223306/widget.png)](https://discord.gg/jjDrGhrjMy)
[![Open in Visual Studio Code](https://open.vscode.dev/badges/open-in-vscode.svg)](vscode://ms-vscode-remote.remote-containers/cloneInVolume?url=https://gitlab.com/aruw/controls/taproot)

# Taproot

A friendly control library and framework for [RoboMaster](https://www.robomaster.com/en-US) robots.

Taproot provides a comprehensive, convenient and well-tested set of APIs designed specifically for
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

Taproot is a C++ library targeting modern C++ (C++17).

We use:
- `modm`, a C++-native HAL
- the gcc compiler
- openocd to deploy and debug
- VSCode, an editor

This library was originally designed for use in Advanced Robotics at the University of Washington
("ARUW")'s robot controls code, which remains the largest consumer. You can find the project
[here](https://gitlab.com/aruw/controls/aruw-mcb).

__In addition to this readme, check out 
[our GitLab wiki](https://gitlab.com/aruw/controls/taproot/-/wikis/home),
[generated documentation](https://aruw.gitlab.io/controls/taproot/) and the other resources linked
below!__


## Getting started

Check out the [template project](https://gitlab.com/aruw/controls/taproot-template-project) to help
you set up a new project. Refer to the resources below for other project information.

See ["Development guide"](#Development-guide) below for information on contributing to this project.

## Resources
- The [generated API documentation for Taproot](https://aruw.gitlab.io/controls/taproot/)
- [aruw-mcb](https://gitlab.com/aruw/controls/aruw-mcb), ARUW's full robot code project
- [aruw-edu](https://gitlab.com/aruw/controls/aruw-edu), a hands-on tutorial for building robot code with Taproot
- [taproot-examples](https://gitlab.com/aruw/controls/taproot-examples), small self-contained code examples
- The [template project](https://gitlab.com/aruw/controls/taproot-template-project) to kick-start your own development
- The [Taproot wiki](https://gitlab.com/aruw/controls/taproot/-/wikis/home)
- The [modm website](https://modm.io/) and associated documentation

## Requirements for use

- A robot operated by a [RoboMaster Development Board Type A](https://store.dji.com/product/rm-development-board-type-a) or [Type C](https://www.robomaster.com/en-US/products/components/general/development-board-type-c/info).
- A Linux, macOS or Windows computer
- An ST-Link- or J-Link-compatible probe for flashing and debugging

## Contacting

If you have any questions please file an Issue on GitLab or join our Discord server (linked above).
We can also be reached privately at robomstr@uw.edu.

## Licensing

Taproot is covered under the GPL-3.0-or-later with the following exceptions:
- The `/modm` submodule is licensed under MPL 2.0 by the modm project. We _are not_
  the license holder for these files. See `/modm/LICENSE` for license information.
- `src/taproot/algorithms/MahonyAHRS.h` and `src/taproot/algorithms/MahonyAHRS.cpp` are licensed
  under the GPL by SOH Madgwick. The repo containing this code can be found
  [here](https://github.com/uw-advanced-robotics/MahonyAHRS).

Other RoboMaster teams are invited, and encouraged, to utilize this library. We have licensed
Taproot and the template project under the GPL to encourage collaboration and open publishing of
RoboMaster controls codebases. We politely request that other teams choosing to utilize this
library, or parts of it (including its design), open-source their own code in turn.

## Contributing

We welcome [Merge Requests](https://gitlab.com/aruw/controls/taproot/-/merge_requests) and
[Issues](https://gitlab.com/aruw/controls/taproot/-/issues)! Feel free to take on an open Issue if
you see one valuable to you. We recommend posting to let us know what you're working on so we don't
duplicate effort.

All development happens on GitLab (not GitHub).

Issues and our Discord server are both great ways to get in touch.

When you create a new branch, always branch off of `develop`.

To learn about contributing to upstream repositories via forks, see here: https://docs.gitlab.com/ee/user/project/repository/forking_workflow.html.

## Development guide

If you are looking to develop _your own_ Taproot-based project, refer to the README there for setup
instructions. The below is for developing Taproot itself. The instructions are very similar but may
be customized per project.

### System setup

Follow the guide appropriate for your operating system.
- Linux
  - Debian: https://gitlab.com/aruw/controls/taproot/-/wikis/Debian-Linux-Setup
  - Fedora: https://gitlab.com/aruw/controls/taproot/-/wikis/Fedora-Linux-Setup
  - Other: follow one of the above guides, substituting your distribution's package names in place
    of Debian or Fedora packages.
- macOS: https://gitlab.com/aruw/controls/taproot/-/wikis/macOS-Setup
- Windows: https://gitlab.com/aruw/controls/taproot/-/wikis/Windows-Setup

Then install `pipenv` and set up the build tools:

```
pip3 install pipenv
cd test-project/
pipenv install
```

Alternately, you want the easiest setup experience and **_do not_ require deploying code to
hardware**, consider developing within the provided [Docker container](https://gitlab.com/aruw/controls/taproot/-/wikis/Docker-Container-Setup).
If you have Docker and vscode installed, you can access this environment in one click using the
badge at the top of this repo. **We do not recommend this approach for robot development.**

Sometimes setting up your machine can be tricky. If you are having trouble setting up your
environment, feel free to ask for help on our [Discord server](https://discord.gg/jjDrGhrjMy).

### Working on Taproot

Most of the time, you want to interact with Taproot from within a project that uses it. Taproot
itself is just a library of code templates; it becomes a fully-fledged buildable entity only when
you configure it and generate an instance of it for your usage.

See [here](https://gitlab.com/aruw/controls/taproot/-/wikis/Code-Generation-in-User-Projects) for
more details on this flow.

For these reasons, the typical way to work with Taproot is from within the project you use it in.
You can edit the top-level `taproot` submodule and re-run `lbuild build` to update your in-project
copy of Taproot according to the modifications you made to the templates.

If you make changes to your `taproot` submodule and want to keep them, you'll need to commit it to
a branch. Unless you have "push" access to the main Taproot repo, you'll need to:
- Fork Taproot under your own name on GitLab
- Push your changes to that fork
- Either [open an MR](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
  for those changes against our repo, or change your own project to use your fork's version

This is not a trivial process, but please do let us know on Discord if you'd like some help!

If you _aren't_ using Taproot within your own project, or would like to avoid the workflow above,
this repo includes a dummy project in the `test-project` folder. You can run `lbuild build` within
that directory to get a copy of Taproot generated locally. This copy supports building and running
tests like any other.

### Branch naming conventions in the Taproot repository

Names should follow the format `FirstL/#{Issue Number}/short-description`. For example:
`RyanT/#0/linter-integration`.

### Building and testing via the terminal

This library is configurable via `lbuild` parameters, and consumers use this tool to generate a full
copy of the files in Taproot in their own projects. As such, there is no "one true configuration"
for consumers using Taproot. See [here](https://gitlab.com/aruw/controls/taproot/-/wikis/Code-Generation-in-User-Projects)
for more details on this flow.

To facilitate testing, this repo has a project defined in `test-project/` which consumes Taproot. It
can be used for basic testing and as a generation/build smoke-test. Note that changes made by ARUW
members should typically be tested in the context of `aruw-mcb` before being merged here.

To use the test project, `cd` into `taproot/test-project` (where the `project.xml` file is). Run
`pipenv shell` to enter the environment with appropriate Python build tools. Then, run
`lbuild build` as described below to generate an Taproot distribution. Then use `scons` to perform
the desired builds.

The `test-project` is a normal Taproot user project, except it configures Taproot to generate its
own unit tests into the project. This means that running `test-project`'s tests runs the tests for
Taproot itself against the version of Taproot generated for the `test-project`.

Likely commands are as follows (all from within a `pipenv shell` or prefixed with `pipenv run <command>`):

- `lbuild build`: Re-generates our copy of modm according to the modules specified in `project.xml`.
- `scons build`: Builds the firmware image for the hardware target. Creates a "release" folder
  located in `build/hardware/` which contains the final `.elf` file as well as the intermediate
  object files (`.o`).
- `scons build-tests`: Builds a program which hosts our unit tests. This executable can be run on
  your host computer and prints results for each unit test run.
- `scons run`: Builds as with `scons build` and then programs the board.
- `scons run-tests`: Builds and runs the unit test program. In `test-project`, this includes all of
  the unit tests for Taproot itself. Same as `build-tests` but also runs the built file.
- `scons size`: Prints statistics on program size and (statically-)allocated memory. Note that the
  reported available heap space is an upper bound, and this tool has no way of knowing about the
  real size of dynamic allocations.

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
`taproot/build_tools/project.xml.in` and add an entry to the dependencies section like the
following:

```xml
<module>modm:platform:gpio</module>
```

Now open the terminal, `cd` into `test-project`, and run `lbuild build`.
