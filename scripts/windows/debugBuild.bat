@echo off
echo Changing directory into project...
cd mcb-2019-2020-project

echo Activating Anaconda...
call activate.bat modm || (
    echo Anaconda not found, try double-checking your PATH entry? Also try restarting the program.
    exit /b 1
)


echo Building...
call scons build profile=debug || (
    echo Something went wrong building, look at the log and try again...
    exit /b 1
)

echo Done!