# mcb-2019-2020

All MCB code for the 2020 RoboMaster competition, using the modm framework, gnu
compiler, openocd debugger, and vscode editor.

## New user guide

1. Download [openocd](https://sourceforge.net/projects/openocd/files/openocd/0.10.0/),
   [arm-gcc-toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads),
   and [anaconda](https://www.anaconda.com/distribution/). If you perfer, rather
   than downloading openocd and the arm-gcc-toolchain from the provided links, I
   have created a 7zip file containing both, which can be downloaded
   [here](https://drive.google.com/file/d/1-GCnAhZSidhW827O36aBPIegsX6G6-6S/view?usp=sharing).
   To download the 7zip files, if you do not have a 7zip extractor, you can use
   [this](https://extract.me/) website to extract the file.
2. Add the /bin of openocd and the gcc-toolchain to your path.

![path-images.PNG](https://i.imgur.com/ZpV4WpX.png)

3. When conda is installed run the following commands in the anaconda prompt: <br>
```
conda create --name modm python=3 pip
activate modm
conda install -c conda-forge git
pip install jinja2 scons future pyelftools lbuild
```
4. Next, clone this repository. When you clone this repository. Insure you type
   `--recursive` when cloning. Your git clone command should look something like
   this: `git clone --recursive
   https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020.git`
   (if you do not use the command `--recursive`, you will have to clone the modm
   submodule. From the mcb-2019-2020 file, in the anaconda prompt, cd into
   mcb-2019-2020/modm and type the following commands: `git submodule init`, 
`git submodule update`)

This will clone the modm subdirectory.
5. Restart your computer for anaconda to properly install and for the path
   variables to be properly updated.

### Instructions for setting up vscode
1. Install the c/c++ extension.
2. Install the cortex-debug extension.
3. For vscode intellisense to work properly, you need to add your compiler path
   individually. To do so, press "ctrl + p" and type `>c/c++ edit configurations
   (UI)`. In the screen that comes up, under the compiler path section, add the
   path of the "arm-none-eabi-gcc.exe" file located in the arm-gcc-toolchain
   /bin folder. Make sure to put double quotation marks around the path name.

## Workflow guide

### Everyday things to know

#### Modm examples

The modm website provides a great number of examples that can be very useful
when implementing something for the first time. The examples are located on
modm's website [here](https://modm.io/#examples).

#### Getting around VSCode

[Here](https://code.visualstudio.com/docs/getstarted/tips-and-tricks) is a very
helpful website that has a number of helpful shortcuts for getting around
vscode. There are many shortcuts that make programming faster.

#### How to build code

1. In anaconda, cd into mcb-2019-2020/mcb-2019-2020-project.
2. In anaconda, type `scons build`.

#### How to add your own files

1. Find the src directory, located at the same level as main.cpp
2. You should create .cpp and .hpp files in the same directories. Choose the
   subdirectory in src that is best suited for file you are creating.

#### How to program an MCB

1. In anaconda, cd into mcb-2019-2020/mcb-2019-2020-project.
2. In anaconda, type `scons program`.

#### How to debug

1. In anaconda, cd into mcb-2019-2020/mcb-2019-2020-project.
2. In anaconda, type `scons build profile=debug`. This will create a debug
   folder in the build folder.
3. In anaconda, type `scons program profile=debug`. This will deploy using the
   newly create debug folder.
4. Open the folder "mcb-2019-2020" in vscode. Hit the debug tab on the left
   side. or type "ctrl + shift + D".
5. Hit the green play arrow on the left top of the screen.

![debug-image](https://i.imgur.com/l78vKh0.png)

### Special cases guide

#### How to add additional modm modules to the modm directory in mcb-2019-2020-project:

1. Open `mcb-2019-2020/mcb-2019-2020-project/project.xml`.
2. Here you will see a list of items that look like this:
   `<module>modm:platform:gpio</module>`. See modm
   [examples](https://modm.io/#examples), where .xml files can be found. Add in
   items you would like. you must enclose the specified item in `<module>
   </module>`. Also, the first word enclosed will most likely be modm. Generally
   speaking, the next word (platform in the example above) signifies the file in
   the modm submodule's 'src' file. This, however, is not always true, and it
   can sometimes be difficult to find what exactly to type to include something
   from the modm submodule. This is why looking at modm's examples are very
   useful because they include project.xml files. 
2. `cd` into mcb-2019-2020-project.
3. type `lbuild build` The dependencies should be created in
   mcb-2019-2020/mcb-2019-2020-project/modm subdirectory.

## Useful Commands when using Anaconda

For these commands to work properly, your anaconda prompt must be in the
mcb-2019-2020/mcb-2019-2020-project directory.

`lbuild build` references the project.xml file to construct files specified in
the .xml file from the modm directory

`scons build` builds the program. Creates a "release" folder located in the
build folder which contains the .elf file and the compiled .o files

`scons build profile=debug` builds the program for a debug configuration.
Doing this creates a folder labeled "debug" located in the build folder.

`scons program` builds and programs the board, using the release version

`scons program profile=debug` builds and programs the board, using debug version