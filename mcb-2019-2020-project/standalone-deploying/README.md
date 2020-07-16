# Standalone programming of the MCB via Openocd

## Prerequisites

- Download the latest version of [openocd](https://gnutoolchains.com/arm-eabi/openocd/). Download the newest version of the software and extract the files to a permanent location. The downloaded file is of type `.7zip`, so you will need a way to extract 7zip files, either by installing the 7zip extractor or by using an online unzipper.
- Add the path of the `/bin` directory of OpenOCD to your PATH environment variable.
    - In your Start menu, search for "environment" and select "Edit the system environment variables."

    - In the "User variables for _{your username here}_", where _{your username here}_ is a placeholder for your actual name. Double click on "Path."<br><br>
    <img src="https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/49592ade19d3a60bb0377d1ae54fd3e6/EnvironmentVarsMain.jpg" height="500px">

    - In "Edit environment variable," click "New."<br><br>
    <img src="https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/8979edf979083ace722dba00883ad153/EnvironmentVarsPathVar.jpg" height="500px">

    - Add the absolute path of the `/bin` directory of OpenOCD to the box that shows up after you click "New."<br><br>
    <img src="https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020/uploads/171f2c159ee4a765680aa7222627a7ec/EnvironmentVarsPathAddition.jpg" height="500px">

    - Select "OK".
- Download the ST-Link V-2 driver [here](https://drive.google.com/drive/u/1/folders/1Ndk8Q-uUtzo3sQtzOoguDAVBZSM3IKT1), unzip, and run the executable to install the driver.

## How to deploy to the MCB

- Download `openocd.cfg` (the file located in this directory).
- In a terminal (bash or windows based), navigate to the folder containing the downloaded `openocd.cfg`
- Type the following:
    ```
    openocd -f "./openocd.cfg" -c "modm_program path/to/elf/file/elf-file-name.elf"
    ```
   where `path/to/elf/file` is a valid path to a `.elf` file that is present, `elf-file-name.elf` is a valid `.elf` file (see below), and `./openocd.cfg` is the file you downloaded in the first step. Note that as of now, downloading from GitLab may mean the filename is changed. You should replace `./openocd.cfg` with the actual name of the file you downloaded.

## Creating an elf file

To get a `.elf` file, you must build the codebase. Refer to [the main repo](#how-to-build-code-and-program-the-mcb) if you are confused about how to build. If you don't have the build environment set up but need a new build, ask on slack for someone to send you a `.elf` file. If you do have the environment but want to deploy using this technique, once you have a build, find the file in the build directory `build/hardware/<debug/hardware>/mcb-2019-2020.elf`. This is the `.elf` file you will want to use while deploying to the MCB.