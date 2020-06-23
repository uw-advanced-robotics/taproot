#include <aruwlib/rm-dev-board-a/board.hpp>

/* arch includes ------------------------------------------------------------*/
#include <aruwlib/architecture/periodic_timer.hpp>

/* communication includes ---------------------------------------------------*/
#include <aruwlib/Drivers.hpp>
#include <aruwlib/display/sh1106.hpp>

/* error handling includes --------------------------------------------------*/

/* control includes ---------------------------------------------------------*/
#include "aruwsrc/control/robot_control.hpp"

using namespace modm::literals;
using aruwlib::Drivers;

/* define timers here -------------------------------------------------------*/
aruwlib::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
void initializeIo();
// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
void updateIo();

int main()
{
    Board::initialize();
    initializeIo();
    aruwsrc::control::initSubsystemCommands();

    while (1)
    {
        // do this as fast as you can
        updateIo();

        if (sendMotorTimeout.execute())
        {
            Drivers::mpu6500.read();
            Drivers::errorController.update();
            Drivers::commandScheduler.run();
            Drivers::djiMotorTxHandler.processCanSendData();
        }
#ifndef ENV_SIMULATOR
        modm::delayMicroseconds(10);
#endif
    }
    return 0;
}

void initializeIo()
{
    aruwlib::Drivers::analog.init();
    aruwlib::Drivers::pwm.init();
    aruwlib::Drivers::digital.init();
    aruwlib::Drivers::leds.init();
    aruwlib::Drivers::can.initialize();

#ifndef ENV_SIMULATOR
    /// \todo this should be an init in the display class
    Board::DisplaySpiMaster::
        connect<Board::DisplayMiso::Miso, Board::DisplayMosi::Mosi, Board::DisplaySck::Sck>();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    Board::DisplaySpiMaster::initialize<Board::SystemClock, 1406250_Hz>();
#endif
    aruwlib::display::Sh1106<
#ifndef ENV_SIMULATOR
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
#endif
        128,
        64,
        false>
        display;
    display.initializeBlocking();

    Drivers::remote.initialize();
    Drivers::mpu6500.init();
    Drivers::refSerial.initialize();
    Drivers::xavierSerial.initialize();
}

void updateIo()
{
    Drivers::canRxHandler.pollCanData();
    Drivers::xavierSerial.updateSerial();
    Drivers::refSerial.updateSerial();
    Drivers::remote.read();
}
