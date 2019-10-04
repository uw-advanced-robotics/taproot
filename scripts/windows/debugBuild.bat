@echo off
echo Changing directory into project...
cd mcb-2019-2020-project
if exist %AppData%\..\Local\Continuum\anaconda3\Scripts\activate.bat (
    echo Activating Anaconda...
    call %AppData%\..\Local\Continuum\anaconda3\Scripts\activate.bat modm
    echo Building...
    scons build profile=debug
    echo Done!
) else if exist %Anaconda%\Scripts\activate.bat (
    echo Activating Anaconda from overide location %Anaconda%...
    call %Anaconda%\Scripts\activate.bat modm
    echo Building...
    scons build profile=debug
    echo Done!
)
else (
    echo Anaconda not found, please try again. (Expected location: %AppData%\..\Local\Continuum\anaconda3\Scripts\activate.bat or %Anaconda%\Scripts\activate.bat)
    exit /B 1
)