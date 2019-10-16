@echo off
echo Building and programming... (Please wait a minute...)
call conda run -n modm scons program profile=debug || (
    echo Something went wrong building, look at the log and try again...
    exit /b 1
)

echo Done!