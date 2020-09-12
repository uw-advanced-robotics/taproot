# aruw-mcb

ARUW's "Main Control Board" (MCB) code for the RoboMaster competition.

The MCB is a [RoboMaster Development Board Type A](https://store.dji.com/product/rm-development-board-type-a)
which directly operates all major systems on our robots. Among these are drive, turret, launcher
wheels, and human input.

Software we use:
- `modm`, a C++-native HAL
- the gcc compiler
- openocd to deploy and debug
- VSCode, an editor

[[_TOC_]]

# Licensing

aruw-mcb is covered under the GPL-3.0-or-later with the following exceptions:
- `/modm` and `/aruw-mcb-project/modm` are licensed under MPL 2.0 by the modm project. We _are not_
  the license holder for these files. See `/modm/LICENSE` for license information.
- `aruw-mcb-project/src/aruwlib/algorithms/MahonyAHRS.h` and
  `/aruw-mcb-project/src/aruwlib/algorithms/MahonyAHRS.cpp` are licensed under the GPL by SOH
  Madgwick. The repo containing this code can be found
  [here](https://github.com/uw-advanced-robotics/MahonyAHRS).

# New user guide

The recommended way to develop is with our pre-built development Docker container.

1. [Install Docker Desktop](https://docs.docker.com/get-docker/). The macOS and Linux instructions will work
   as-is. If you are on Windows, there are two options:
     - If you are on Windows 10 version 2004 or later ([how do I know?](https://support.microsoft.com/en-us/help/13443/windows-which-version-am-i-running)),
       the relevant guide is [here](https://docs.docker.com/docker-for-windows/install-windows-home/).
       Make sure you click "Enable WSL 2 Features" in the Docker installer.
     - If you cannot update to Windows 10 version 2004 (recommended) but are running Windows 10 Pro
       (not Home), you can follow [this guide](https://docs.docker.com/docker-for-windows/install/)
       to use the Hyper-V backend instead.
2. [Install Visual Studio Code](https://code.visualstudio.com/).
3. [Install git](https://git-scm.com/).
4. Open Visual Studio Code.
5. Clone this repo. You can go to the "source control" tab on the left of the editor and choose
   "Clone Repository". When asked, enter `https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020.git`
   as the source URL. Pick a reasonable location to clone the repo into. Make sure you have your gitlab credentials ready when you clone your repo.
   
   If you don't enter them correctly the first time that's okay - If you're on windows open the **Credential Manager** app and go to **Windows Credentials**. Then correct your git credentials stored under `git:https://gitlab.com` before trying to clone again.

   <img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/1e1f271fbda7085856d57e85491eae91/image.png" width="500px">

6. Once it opens the new folder, an alert will appear with suggested extensions. Click "Install All".

   <img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/edb88ab3074a99d5eed1a4b2a50b2e53/image.png" width="500px">

7. vscode will now prompt you to open the development container. Choose "Reopen in Container". This
   step may take a few minutes (or more!); it will download around 500 MB (2GB once uncompressed) of
   necessary tools. Be patient.

   _Heads-up:_ If you don't see the dialog, or it goes away, you can press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>
   and type "Reload Window" plus <kbd>Enter</kbd> to reload vscode.

   <img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/198095587078b88d67ac674839cf4ad7/image.png" width="500px">

   vscode is now attached to a "container", which includes all the tools
   necessary to develop in this repo! In the future, you will always want to open
   the workspace within its container.

Now that you have the environment, let's test it out! Press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>,
type "Focus Next Terminal", and press <kbd>Enter</kbd>. In this terminal, type `scons run-tests` and
press <kbd>Enter</kbd>. After building our code, it should run the tests and print a message
indicating that all tests passed.

### Optional: prevent Docker Desktop from running on startup

By default, Docker will likely be configured to run on startup. Your mileage may vary, but this may
be frustrating. If so, you can disable it! Open the Docker Desktop settings, which on Windows can be
accessed via the Docker icon in the system tray:

<img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/3aaec4ca0336a4b81f1a5d573a18d05f/image.png" width="250px">


The "General" tab has a checkbox for disabling auto-start.

<img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/4b18706432e7d51c8a4d98b00462b399/image.png" width="500px">


In the future, if you attempt to load the repository within the dev container and haven't manually
started Docker, you will get the following error:

<img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/3b03894ff5bc441c8273becfbf4980c6/image.png" width="500px">


In this case, you can launch Docker manually and hit "Retry". On Windows, Docker Desktop can be
started by searching for "docker" in the Start menu.

<img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/64e97a4ff076f79069ae4d8f670e19bd/image.png" width="300px">

**We also recommend you _stop_ Docker when you're done! This can be done on Windows via the same
icon in the system tray. This will help preserve battery life and RAM.**

## Non-containerized setup (NOT RECOMMENDED; see above)

See the [wiki](https://gitlab.com/aruw/controls/aruw-mcb/-/wikis/home) for information about how
to set up your system manually on a Windows or Ubuntu Linux machine. This should only be necessary
if you are unable to use the Docker container provided above and is not recommended because of the
complexity of setting up the environment correctly and because not all features are supported.

## Workflow guide

### Branch naming conventions

- When you create a new branch, always branch off of `develop` (not `master`)
- Names should follow the format `FirstL/{Issue Number}/short-description`
- Example: `RyanT/0/linter-integration`

### Getting around VSCode

Microsoft provides a [helpful website](https://code.visualstudio.com/docs/getstarted/tips-and-tricks) that has a number of helpful shortcuts for getting around VSCode. There are many shortcuts that make programming faster. It is much appreciated when someone asks for help and can quickly navigate through the codebase while we work through a code bug.

### How to build code and program the MCB

_If you would like to use the terminal instead, see the section "Building and running via the terminal" below._

1. Make sure you have VSCode opened in the folder `mcb-2019-2020` (**not `mcb-2019-2020-project`**)
2. Connect an ST-Link to the MCB and your computer.
3. In VSCode, open the Command Palette (<kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>P</kbd>)
4. Find `Tasks: Run Task`. You should see the options below
<br><br>
    <img src=https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/08a6f537b097a6a0930688b8ac1f67a3/vscode-build-image.png height="200px" />

### How to debug using an ST-Link

1. Open the folder `mcb-2019-2020` in VSCode. Hit the debug tab on the left side or type <kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>D</kbd>.
2. Hit the green play arrow on the left top of the screen.
3. See [this page](https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/-/wikis/Software-Tools/Debugging-With-STLink) for more information about using the ST-Link for programming the MCB and debugging.
<br>
<img src=https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/696c468a1b7fe346822984ba4fae1f56/vscode-debug-img.png height="400px" />

### How to debug using a J-Link

See the [wiki](https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/-/wikis/Software-Tools/Debugging-With-JLink) for an explanation on the difference between an ST-Link and J-Link and a step-by-step procedure on how to use the J-Link.

### How to select robot type

With the root directory opened in VSCode, type <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>. Type "ARUW: Select Robot Type" and hit enter. A dropdown menu should appear. Select the robot type from the dropdown.

## Working with modm

### What is modm?

We use an embedded library generator called modm in our codebase. It will eventually be important that you understand what how modm works. For now you can just think about it as handling lower level IO on our MCB. You should read [modm's homepage](https://modm.io/) so you have a general idea of what it does.

### Modm examples

The modm website provides a great number of examples that can be very useful when interacting with modm's hardware architecture layer for the first time. The examples are located on modm's website [here](https://modm.io/#examples).

### Adding new dependencies on modm modules (advanced)

Look up the fully-qualified name of the module from the [modm website](https://modm.io/reference/module/modm-architecture/).
The name will look like `:platform:gpio`. Open `mcb-2019-2020-project/project.xml` and add an entry to the dependencies section like the following:

```xml
<module>modm:platform:gpio</module>
```

Now open the terminal and run `lbuild build`.

# Building and running via the terminal

The below commands require that your working directory is `aruw-mcb/mcb-2019-2020-project` (where the `SConstruct` and `project.xml` files are).

- `lbuild build`: Re-generates our copy of modm according to the modules specified in `project.xml`. Note that there is a _separate_ instance used for the unit tests, which can be build by runnint the same command from within the `sim-modm` subdirectory.
- `scons build`: Builds the firmware image for the hardware target. Creates a "release" folder located in `build/hardware/` which contains the final `.elf` file as well as the intermediate object files (`.o`).
- `scons build-tests`: Builds a program which hosts our unit tests. This executable can be run on your host computer (only supported on Linux) and prints results for each unit test run.
- `scons program`: Builds as with `scons build` and then programs the board.
- `scons run-tests`: Builds and runs the unit test program.
- `scons size`: Prints statistics on program size and (satically-)allocated memory. Note that the reported available heap space is an upper bound, and this tool has no way of knowing about the real size of dynamic allocations.

Note that all `scons` commands have optional `profile` and `target` options; the former controls whether performance and size optimizations are applied to the output, and the latter specifies which robot to build for. The default is to build in release mode for the Soldier.

```
Usage: scons <target> [profile=<debug|release>] [robot=TARGET_<ROBOT_TYPE>]
    "<target>" is one of:
        - "build": build all code for the hardware platform.
        - "run": build all code for the hardware platform, and deploy it to the board via a connected ST-Link.
        - "build-tests": build core code and tests for the current host platform.
        - "run-tests": build core code and tests for the current host platform, and execute them locally with the test runner.
    "TARGET_<ROBOT_TYPE>" is an optional argument that can override whatever robot type has been specified in robot_type.hpp.
        - <ROBOT_TYPE> must be one of the following:
            - SOLDIER, OLD_SOLDIER, DRONE, ENGINEER, SENTINEL, HERO:
```
