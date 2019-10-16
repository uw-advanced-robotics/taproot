@echo off
echo Building and programming...
call conda run -n modm scons program || (
    echo Something went wrong building, look at the log and try again...
    exit /b 1
)

echo Done!