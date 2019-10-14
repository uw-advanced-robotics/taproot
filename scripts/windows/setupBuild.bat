@echo off
echo Changing directory into project...
cd mcb-2019-2020-project

echo Running lbuild...
call conda run -n modm lbuild build || (
    echo Something went wrong while running lbuild, look at the log and try again...
    exit /b 1
)

echo Done!