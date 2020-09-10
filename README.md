# mcb-2019-2020

All ARUW's MCB code for the RoboMaster competition. We use:
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
5. Clone this repo. You can go to the "source control" tab on the right of the editor and choose
   "Clone Repository". When asked, enter `https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020.git`
   as the source URL. Pick a reasonable location to clone the repo into.

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

## Optional: prevent Docker Desktop from running on startup

By default, Docker will likely be configured to run on startup. Your mileage may vary, but this may
be frustrating. If so, you can disable it! Open the Docker Desktop settings, which on Windows can be
accessed via the Docker icon in the system tray:

<img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/3aaec4ca0336a4b81f1a5d573a18d05f/image.png" width="500px">

The "General" tab has a checkbox for disabling auto-start.

<img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/4b18706432e7d51c8a4d98b00462b399/image.png" width="500px">

In the future, if you attempt to load the repository within the dev container and haven't manually
started Docker, you will get the following error:

<img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/3b03894ff5bc441c8273becfbf4980c6/image.png" width="500px">

In this case, you can launch Docker manually and hit "Retry". On Windows, Docker Desktop can be
started by searching for "docker" in the Start menu.

<img src="https://gitlab.com/aruw/code-2019-2020/aruw-mcb-2019-2020/uploads/64e97a4ff076f79069ae4d8f670e19bd/image.png" width="500px">

**We also recommend you _stop_ Docker when you're done! This can be done on Windows via the same
icon in the system tray. This will help preserve battery life and RAM.**

## Non-containerized setup (NOT RECOMMENDED; see above)

See the [wiki](https://gitlab.com/aruw/controls/aruw-mcb/-/wikis/home) for information about how
to set up your system manually on a Windows or Ubuntu Linux machine. This should only be necessary
if you are unable to use the Docker container provided above and is not recommended because of the
complexity of setting up the environment correctly and because not all features are supported.

# Workflow guide

## Branch naming conventions

- When you create a new branch, always branch off of `develop` (not `master`)
- Names should follow the format `FirstL/{Issue Number}/short-description`
- Example: `RyanT/0/linter-integration`

## What is modm?

We use an embedded library generator called modm in our codebase. It will eventually be important that you understand what how modm works. For now you can just think about it as handling lower level IO on our MCB. You should read [modm's homepage](https://modm.io/) so you have a general idea of what it does.

## Modm examples

The modm website provides a great number of examples that can be very useful when interacting with modm's hardware architecture layer for the first time. The examples are located on modm's website [here](https://modm.io/#examples).

## Getting around VSCode

Microsoft provides a [helpful website](https://code.visualstudio.com/docs/getstarted/tips-and-tricks) that has a number of helpful shortcuts for getting around VSCode. There are many shortcuts that make programming faster. It is much appreciated when someone asks for help and can quickly navigate through the codebase while we work through a code bug.

## How to build code and program the MCB

1. Make sure you have VSCode opened in the folder `mcb-2019-2020` (**not `mcb-2019-2020-project`**)
2. Connect an ST-Link to the MCB and your computer.
3. In VSCode, open the Command Palette (<kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>P</kbd>)
4. Find `Tasks: Run Task`. You should see the options below
<br><br>
    <img src=https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/08a6f537b097a6a0930688b8ac1f67a3/vscode-build-image.png height="200px" />

## How to debug using an ST-Link

1. Open the folder `mcb-2019-2020` in VSCode. Hit the debug tab on the left side or type <kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>D</kbd>.
2. Hit the green play arrow on the left top of the screen.
3. See [this page](https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/-/wikis/Software-Tools/Debugging-With-STLink) for more information about using the ST-Link for programming the MCB and debugging.

<br>

<img src=https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/696c468a1b7fe346822984ba4fae1f56/vscode-debug-img.png height="400px" />

## How to debug using a J-Link

See the [wiki](https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/-/wikis/Software-Tools/Debugging-With-JLink) for an explanation on the difference between an ST-Link and J-Link and a step-by-step procedure on how to use the J-Link.

## How to select robot type

With the root directory opened in VSCode, type <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>. Type "ARUW: Select Robot Type" and hit enter. A dropdown menu should appear. Select the robot type from the dropdown.

## Advanced techniques: working with modm modules and lbuild

Below are steps which explain how to add additional modm modules to the modm directory in the mcb-2019-2020-project directory.

1. Open `mcb-2019-2020/mcb-2019-2020-project/project.xml`.
2. Here you will see a list of items that take the following form: `<module>modm<module name></module>`.
3. To add a particular module, first find the module the folder presides in in the modm submodule (you can find the module in the path `modm/src/modm`).
4. Here you should find a `module.lb` file. In the file search for `def init`. For example, if I wanted to add the gpio module, I would find the `module.lb` file with the path `modm\src\modm\platform\gpio\stm32\module.lb`, then find the `init` function, where I would find the following:

    ```python
    def init(module):
        module.name = ":platform:gpio"
        module.description = "General Purpose I/O (GPIO)"
    ```

5. I now have the necessary information to add the module correctly. In my example, the module name is `:platform:gpio`, so I would add to the `project.xml` the following: `<module>modm:platform:gpio</module>`.
6. Run the `Setup Build` task in VSCode, or `lbuild build` in anaconda (with modm activated).

## How to modify modm to run tests on a local machine for Linux

Currently, the modm build that runs locally is set up for windows. To modify this to run on Linux, do the following:
- In a terminal, navigate to `mcb-2019-2020/mcb-2019-2020-project/sim-modm`.
- Open `project.xml` in VSCode (in the terminal, type `code project.xml`).
- You will find the below two lines. Remove the comment around the "hosted-linux" target (`<!-- -->`), and comment out the "hosted-windows" target.
    ```xml
    <option name="modm:target">hosted-windows</option>
    <!-- <option name="modm:target">hosted-linux</option> -->
    ```
- Type `lbuild build` in the terminal you have open.
- `cd ..` and type `scons run-tests` to insure the updates have succeeded.

# Useful commands for running lbuild and scons

For these commands to work properly, your anaconda prompt must be in the `mcb-2019-2020/mcb-2019-2020-project` directory (where the `SConstruct` and `project.xml` files are) and you must have modm activated (`activate modm` on anaconda prompt).

- `lbuild build` references the project.xml file to construct files specified in the .xml file from the modm directory.

- `scons build` builds the program. Creates a "release" folder located in the build folder which contains the `.elf` file and the compiled `.o` files.

- `scons build-tests` builds the unit testing program. As opposed to building to deploy on the mcb, this builds an executable which can be ran on your computer.

- `scons build profile=debug` builds the program for a debug configuration. Doing this creates a folder labeled "debug" located in the build folder.

- `scons program` builds and programs the board, using the release configuration.

- `scons build-tests` builds and runs the unit test program.

- `scons program profile=debug` builds and programs the board, using debug build configuration.
