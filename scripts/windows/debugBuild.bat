@echo off
echo Building...
call conda run -n modm scons build profile=debug || (
    echo Something went wrong building, look at the log and try again...
    exit /b 1
)

echo Done!