@echo off
echo Changing directory into project...
cd mcb-2019-2020-project

echo Building...
call conda run -n modm scons build profile=debug || (
    echo Something went wrong building, look at the log and try again...
    exit /b 1
)

echo Done!