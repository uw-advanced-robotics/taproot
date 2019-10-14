@echo off
echo Changing directory into project...
cd mcb-2019-2020-project

echo Activating Anaconda...
call activate.bat modm || (
    echo Anaconda not found, try double-checking your PATH entry? Also try restarting the program.
    exit /b 1
)


echo Running lbuild...
call lbuild build || (
    echo Something went wrong while running lbuild, look at the log and try again...
    exit /b 1
)

echo Done!