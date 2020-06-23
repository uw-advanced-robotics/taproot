# mcb-2019-2020

All ARUW MCB code for RoboMaster, using the modm framework, gnu compiler, openocd debugger, and VSCode editor.

[[_TOC_]]

# New user guide

## Download and installation guide

Note: The beginning of the guide is for windows users. If you are using mac or linux, skip steps 1-5 and instead refer to [modm's guide here](https://modm.io/guide/installation/).

_**Warning:** This build setup has not been fully tested on linux or mac. Compatibility with mac/linux is being worked on._

1. Download [arm-gcc-toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads). When you open downloads page, you will see a number of installers. Choose the .zip version of newest toolchain release. Extract the contents of the downloaded file to a permanent location of your choice.
2. Download the latest version of [openocd](https://gnutoolchains.com/arm-eabi/openocd/). Download the newest version of the software and extract the files to a permanent location. The downloaded file is of type `.7zip`, so you will need a way to extract 7zip files, either by installing the 7zip extractor or by using an online unzipper.
3. Download and install [anaconda](https://www.anaconda.com/distribution/). During installation of Anaconda, make sure to check the 'Add to PATH' box.
4. Download MinGW. Follow the installation guide [here](http://www.mingw.org/wiki/Getting_Started).
5. Add the path of the `/bin` directories of OpenOCD, the GNU Toolchain, and MinGW to your PATH environment variable. You can find this setting by searching for "environment" in your Start menu and selecting "Edit the system environment variables". Then click "Environment Variables..." and enter the paths as shown below.<br><br>

    <img src="https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/0a2c25d23166bc1ff58b41594ba63e4e/Overview.png" height="500px" />

6. Download the ST-Link V-2 driver [here](https://www.st.com/en/development-tools/stsw-link009.html#get-software), unzip, and run the executable to install the driver.
7. Download [ozone_jlink_jscope.zip](https://drive.google.com/drive/u/1/folders/1E9i1JBILotoFClKc6L3-m4OWuW0kiO5A) in our google drive. This contains installers for Ozone, the J-Link drivers, and J-Scope (all J-Link related software). Once downloaded and unzipped, run the three installers.
8. Download and install [VSCode](https://code.visualstudio.com/download).
9. Open anaconda prompt (type "Anaconda Prompt" into the start menu to start the prompt) and run the following commands: <br>
```
conda create --name modm python=3 pip
activate modm
conda install -c conda-forge git pywin32
pip install jinja2 scons future pyelftools lbuild
```
10. Clone this repository: In a new terminal (Git Bash, anaconda, or cmd should work) type `git clone --recursive https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020.git`.
    - <em>If you forget the `--recursive`, run: `git submodule init .\mcb-2019-2020\modm\` and `git submodule update --recursive`.</em>
11. Restart your computer for anaconda to properly install and for the path variables to be properly updated.

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

# Useful commands for running lbuild and scons

For these commands to work properly, your anaconda prompt must be in the `mcb-2019-2020/mcb-2019-2020-project` directory (where the `SConstruct` and `project.xml` files are) and you must have modm activated (`activate modm` on anaconda prompt).

- `lbuild build` references the project.xml file to construct files specified in the .xml file from the modm directory.

- `scons build` builds the program. Creates a "release" folder located in the build folder which contains the `.elf` file and the compiled `.o` files.

- `scons build-tests` builds the unit testing program. As opposed to building to deploy on the mcb, this builds an executable which can be ran on your computer.

- `scons build profile=debug` builds the program for a debug configuration. Doing this creates a folder labeled "debug" located in the build folder.

- `scons program` builds and programs the board, using the release configuration.

- `scons build-tests` builds and runs the unit test program.

- `scons program profile=debug` builds and programs the board, using debug build configuration.
