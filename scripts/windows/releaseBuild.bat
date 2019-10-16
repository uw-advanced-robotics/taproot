@echo off
echo Building... (Please wait a minute...)
call conda run -n modm scons build || (
    echo Something went wrong building, look at the log and try again...
    exit /b 1
)

echo Done!