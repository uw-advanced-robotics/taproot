# mcb-2019-2020

All ARUW's MCB code for the RoboMaster competition. We use:
- `modm`, a C++-native HAL
- the gcc compiler
- openocd to deploy and debug
- VSCode, an editor

[[_TOC_]]

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

## Non-containerized setup (NOT RECOMMENDED; see above)

For Windows and Debian Linux, there is specific setup that documented in this guide. For Windows users, see the [Windows specific setup](#windows-specific-setup). For Linux specific setup, see [Debian Linux (Ubuntu 18.04 and 20.04 LTS) specific setup](#debian-linux-ubuntu-18.04-and-20.04-lts-specific-setup). For mac, instead refer to [modm's guide here](https://modm.io/guide/installation/). Once operating system specific setup has been complete, jump to [operating system independent setup](#operating-system-independent-setup) to complete the necessary setup.

_**Warning:** This build setup has not been fully tested on mac. Compatibility with mac is being worked on._

### Windows specific setup

1. Download [arm-gcc-toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads). When you open downloads page, you will see a number of installers. Choose the .zip version of newest toolchain release. Extract the contents of the downloaded file to a permanent location of your choice.
2. Download the latest version of [openocd](https://gnutoolchains.com/arm-eabi/openocd/). Download the newest version of the software and extract the files to a permanent location. The downloaded file is of type `.7zip`, so you will need a way to extract 7zip files, either by installing the 7zip extractor or by using an online unzipper.
3. Download and install [anaconda](https://www.anaconda.com/distribution/). During installation of Anaconda, make sure to check the 'Add to PATH' box.
4. Download MinGW. Follow the installation guide [here](http://www.mingw.org/wiki/Getting_Started).
5. Add the path of the `/bin` directories of OpenOCD, the GNU Toolchain, and MinGW to your PATH environment variable.
    - In your Start menu, search for "environment" and select "Edit the system environment variables."

    - In the "User variables for _{your username here}_", where _{your username here}_ is a placeholder for your actual name. Double click on "Path."<br><br>
    <img src="https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/49592ade19d3a60bb0377d1ae54fd3e6/EnvironmentVarsMain.jpg" height="500px">

    - In "Edit environment variable," click "New."<br><br>
    <img src="https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/8979edf979083ace722dba00883ad153/EnvironmentVarsPathVar.jpg" height="500px">

    - Add the absolute path of the `/bin` directory of one of the tools listed above to the box that shows up after you click "New". Repeat this process for all of the necessary paths listed above.<br><br>
    <img src="https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/171f2c159ee4a765680aa7222627a7ec/EnvironmentVarsPathAddition.jpg" height="500px">

    - Select "OK."
6. Download the ST-Link V-2 driver [here](https://drive.google.com/drive/u/1/folders/1Ndk8Q-uUtzo3sQtzOoguDAVBZSM3IKT1), unzip, and run the executable to install the driver.
7. Download [ozone_jlink_jscope.zip](https://drive.google.com/drive/u/1/folders/1E9i1JBILotoFClKc6L3-m4OWuW0kiO5A) in our google drive. This contains installers for Ozone, the J-Link drivers, and J-Scope (all J-Link related software). Once downloaded and unzipped, run the three installers.
8. Open anaconda prompt (type "Anaconda Prompt" into the start menu to start the prompt) and run the following commands: <br>
    ```
    conda create --name modm python=3 pip
    activate modm
    conda install -c conda-forge git pywin32
    pip install jinja2 scons future pyelftools lbuild
    ```

### Debian Linux (Ubuntu 18.04 and 20.04 LTS) specific setup

1. Run the following commands in a terminal:
    ```
    sudo apt-get install python3 python3-pip scons git
    sudo apt-get --no-install-recommends install doxygen
    pip3 install modm
    sudo apt-get install gcc-avr binutils-avr avr-libc avrdude
    sudo apt-get install openocd
    pip3 install gdbgui
    sudo apt-get install gcc build-essential libboost-all-dev
    pip3 install lbuild
    pip3 install pyelftools
    ```
2. Add lbuild to your path:
    - Open `~/.bashrc` with a text editor.
    - Scroll down to the end of the file and type this:
        ```
        PATH="$HOME/.local/bin:$PATH"
        export PATH
        ```
    - Save the new updates.
3. Download the [latest version](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) of the arm-none-eabi toolchain. In particular, download "gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2", and unzip the file (`tar -xvf gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2`).
4. Add the `/bin` folder of `gcc-arm-none-eabi-9-2020-q2-update` to your path:
    - Open `~/.bashrc` with a text editor.
    - Scroll down to the end of the file, and right above `PATH="$HOME/.local/bin:$PATH"`, type this:
        ```
        PATH=path/to/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH
        ```
       Where `path/to/gcc-arm-none-eabi-9-2020-q2-update` is replaced with your own path to the toolchain.

14. When running on Ubuntu 18.04 LTS, an additional step must be taken to insure python 3 is used. If you are running 18.04, for a less intrusive way to run all scons scripts with Python 3 add this to your `.bashrc` or `.zshrc`:
    ```
    alias scons="/usr/bin/env python3 $(which scons)"
    ```
- Download and compile the latest version of openocd (`apt-get install openocd` is not enough by itself).
    - In particular, you must clone the [openocd git repo](https://github.com/ntfreak/openocd), then run the following commands in a terminal located at the root directory of the cloned repo (taken from the openocd [README](https://github.com/ntfreak/openocd/blob/master/README)):
        ```
        ./bootstrap
        ./configure
        make
        sudo make  install
        ```
- Install the ST-Link V2 driver. This is done through the terminal. The following instructions were taken and partially modified from [this site](https://freeelectron.ro/installing-st-link-v2-to-flash-stm32-targets-on-linux/).
    ```
    sudo apt-get install git make cmake libusb-1.0-0-dev
    sudo apt-get install gcc build-essential
    cd
    mkdir stm32
    cd stm32
    git clone https://github.com/texane/stlink
    cd stlink
    cmake .
    make
    sudo make install
    cd bin
    sudo cp st-* /usr/local/bin
    cd ..
    sudo cp config/udev/rules.d/49-stlinkv* /etc/udev/rules.d/ 
    sudo udevadm control --reload
    ```
- If you have an ST-Link on hand (if you are in lab, ask someone to help find one), test if the installation worked correctly. See the end of the document mentioned in the previous step to insure the ST-Link driver is properly installed.
- **Once you have cloned the mcb-2019-2020 repository (see below)**, in a terminal navigate to `mcb-2019-2020-project` and type `lbuild build`. This will regenerate some of modm's code so you can build in Linux. Similarly, see [below](#how-to-modify-modm-to-run-tests-on-a-local-machine-for-linux) for how to update the source files for building the simulator on Linux.

### Operating system independent setup

1. Download and install [VSCode](https://code.visualstudio.com/download).
2. Clone this repository: In a new terminal (Git Bash, anaconda, or cmd should work) type `git clone --recursive https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020.git`.
    - _If you forget the `--recursive`, run: `git submodule init .\mcb-2019-2020\modm\` and `git submodule update --recursive`._
3. Restart your computer to insure all installs have been updated.

## VSCode setup guide

1. Open the `mcb-2019-2020` folder in VSCode. The first time you try to open this folder, you can start VSCode, then in the "Welcome" page, under the "Start" menu, find "Open folder...". **Important: do NOT open the `mcb-2019-2020-project` folder.** If you do you will not be able to build code and VSCode intellisense will not work as expected.
2. When you open up the folder, VSCode should prompt you to install the recommended extensions. Click "install recommended extensions". You should now have the "C/C++", "Cortex-Debug", and "aruw robot chooser" extensions.
    - If you do not, in extensions: marketplace (to open, type <kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>X</kbd>), search "@recommended" and install all under "WORKSPACE RECOMMENDED".
3. For VSCode intellisense to work properly, you need to add your compiler path to the .vscode project folder. To do so, type <kbd>ctrl</kbd>+<kbd>P</kbd>, type `>c/c++ edit configurations (JSON)`, and hit enter. This will generate a `c_cpp_properties.json` file with some default settings. Replace the contents of this file with the below json. Edit the "compilerPath" setting so it matches the example below. In particular, the "compilerPath" option should be the absolute path name to your version of `arm-none-eabi-gcc.exe` that you just installed above, which will be located in the arm-gcc-toolchain's `/bin` folder.
```json
{
    "configurations": [
        {
            "name": "Win32",
            "compilerPath": "C:/path/to/bin/arm-none-eabi-gcc.exe"
        }
    ],
    "version": 4
}
```

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
